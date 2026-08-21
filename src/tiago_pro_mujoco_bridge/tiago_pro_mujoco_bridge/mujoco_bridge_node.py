#!/usr/bin/env python3
"""MuJoCo-based simulated hardware bridge for Tiago Pro.

Stands in for the real robot on the tiago_pro_opensot_node / cartesian_interface_node
pipeline: publishes /joint_states from a live MuJoCo simulation and drives the
simulated torso + arm actuators from the OpenSoT solver's /opensot/joint_states output.

The tiago-pro-mujoco XML is fixed-base and headless (no wheel or head joints), so
base and head commands from OpenSoT are simply not applied here.
"""
import time
import array

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

from tiago_control_node.utils import tiago_pro_home_config

# Joint name -> MuJoCo position actuator name (only the DoFs this MuJoCo model actuates)
JOINT_TO_MOTOR = {
    'torso_lift_joint': 'torso_lift_motor',
    **{f'arm_left_{i}_joint': f'arm_left_{i}_motor' for i in range(1, 8)},
    **{f'arm_right_{i}_joint': f'arm_right_{i}_motor' for i in range(1, 8)},
}

# Measured empirically in MuJoCo: 0.0 rad -> ~102mm fingertip gap (open), 0.9 rad -> ~7mm gap (closed).
# This is the OPPOSITE of tiago_pro_sim.py's comment - verified directly against this XML's linkage.
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSED_POS = 0.9

# tiago_pro_home_config layout: [floating_base(7), wheels(8), torso(1), arm_left(7), arm_right(7), head(2)]
HOME_POSITIONS = {
    'torso_lift_joint': tiago_pro_home_config[15],
    **{f'arm_left_{i}_joint': tiago_pro_home_config[15 + i] for i in range(1, 8)},
    **{f'arm_right_{i}_joint': tiago_pro_home_config[22 + i] for i in range(1, 8)},
}


class TiagoProMujocoBridge(Node):
    def __init__(self):
        super().__init__('tiago_pro_mujoco_bridge')

        self.declare_parameter('mujoco_xml_path',
            '/home/forest_ws/robots/pal_tiago_pro/xmls/scene_tiago_pro.xml')
        self.declare_parameter('viewer', True)
        self.declare_parameter('fps', 60.0)
        self.declare_parameter('command_topic', '/opensot/joint_states')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('gripper_speed', 0.8)
        self.declare_parameter('target_object_joint', 'cube_freejoint')
        self.declare_parameter('log_path', '')

        xml_path = self.get_parameter('mujoco_xml_path').value
        self.use_viewer = self.get_parameter('viewer').value
        self.fps = self.get_parameter('fps').value
        command_topic = self.get_parameter('command_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        target_object_joint = self.get_parameter('target_object_joint').value
        self.log_path = self.get_parameter('log_path').value

        self.get_logger().info(f"Loading MuJoCo model: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.steps_per_render = max(1, int(round((1.0 / self.fps) / self.model.opt.timestep)))

        # Resolve joint/actuator addresses once
        self.qpos_adr = {}
        self.dof_adr = {}
        self.actuator_id = {}
        for joint_name, motor_name in JOINT_TO_MOTOR.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_name)
            if jid == -1 or aid == -1:
                self.get_logger().warn(f"'{joint_name}' / '{motor_name}' not found in MuJoCo model, skipping.")
                continue
            self.qpos_adr[joint_name] = self.model.jnt_qposadr[jid]
            self.dof_adr[joint_name] = self.model.jnt_dofadr[jid]
            self.actuator_id[joint_name] = aid

        self.joint_names = list(self.actuator_id.keys())
        self.target_positions = dict(HOME_POSITIONS)

        # Gripper fingers are actuated directly (not driven by /opensot/joint_states)
        self.gripper_actuator_id = {}
        for side in ('left', 'right'):
            for finger in ('left', 'right'):
                name = f'gripper_{side}_finger_{finger}'
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                if aid == -1:
                    self.get_logger().warn(f"Actuator '{name}' not found in MuJoCo model, skipping.")
                    continue
                self.gripper_actuator_id[name] = aid
        self.gripper_status = {'left': 'open', 'right': 'open'}
        self.current_gripper_pos = {'left': GRIPPER_OPEN_POS, 'right': GRIPPER_OPEN_POS}

        # End-effector sites and the logged target object (the cube), for get_log_entry()
        self.eef_site_id = {}
        for side in ('left', 'right'):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'ee_{side}')
            if sid == -1:
                self.get_logger().warn(f"Site 'ee_{side}' not found in MuJoCo model; eef logging for {side} disabled.")
            self.eef_site_id[side] = sid

        tjid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, target_object_joint)
        if tjid == -1:
            self.get_logger().warn(f"Joint '{target_object_joint}' not found; target_object_pose logging disabled.")
            self.target_object_qpos_adr = None
        else:
            self.target_object_qpos_adr = self.model.jnt_qposadr[tjid]

        self.log_buffer = []

        self._reset_to_home()

        self.command_sub = self.create_subscription(
            JointState, command_topic, self._command_cb, 10)
        self.joint_state_pub = self.create_publisher(
            JointState, joint_states_topic, 10)

        for side in ('left', 'right'):
            self.create_subscription(
                Bool, f'/mujoco_bridge/gripper_{side}/open',
                lambda msg, s=side: self._gripper_cmd_cb(s, msg), 10)

        self.get_logger().info(
            f"Tiago Pro MuJoCo bridge ready. Driving {len(self.joint_names)} joints "
            f"from '{command_topic}', publishing state on '{joint_states_topic}'.")

    def _gripper_cmd_cb(self, side, msg: Bool):
        self.gripper_status[side] = 'open' if msg.data else 'close'

    def _reset_to_home(self):
        mujoco.mj_resetData(self.model, self.data)
        for joint_name, pos in HOME_POSITIONS.items():
            if joint_name not in self.qpos_adr:
                continue
            self.data.qpos[self.qpos_adr[joint_name]] = pos
            self.data.ctrl[self.actuator_id[joint_name]] = pos
        for name, aid in self.gripper_actuator_id.items():
            self.data.ctrl[aid] = GRIPPER_OPEN_POS
        mujoco.mj_forward(self.model, self.data)

    def actuate_gripper(self, side, status):
        """Ramps one gripper smoothly towards open/closed. Call once per render frame."""
        if side not in ('left', 'right') or status not in ('open', 'close'):
            return

        target = GRIPPER_OPEN_POS if status == 'open' else GRIPPER_CLOSED_POS
        dt = self.model.opt.timestep * self.steps_per_render
        current = self.current_gripper_pos[side]

        if current < target:
            current = min(current + self.gripper_speed * dt, target)
        elif current > target:
            current = max(current - self.gripper_speed * dt, target)

        self.current_gripper_pos[side] = current
        for finger in ('left', 'right'):
            aid = self.gripper_actuator_id.get(f'gripper_{side}_finger_{finger}')
            if aid is not None:
                self.data.ctrl[aid] = current

    def actuate_grippers(self):
        for side in ('left', 'right'):
            self.actuate_gripper(side, self.gripper_status[side])

    def _command_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_positions:
                self.target_positions[name] = pos

    def apply_targets(self):
        for joint_name, aid in self.actuator_id.items():
            self.data.ctrl[aid] = self.target_positions[joint_name]

    def step_physics(self):
        for _ in range(self.steps_per_render):
            mujoco.mj_step(self.model, self.data)

    def get_log_entry(self) -> dict:
        """Snapshots the current sim state for offline logging (camera streams excluded)."""
        entry = {}
        for side in ('left', 'right'):
            sid = self.eef_site_id[side]
            if sid == -1:
                continue
            pos = self.data.site_xpos[sid].copy()
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, self.data.site_xmat[sid])
            entry[f'eef_{side}_pos'] = pos
            entry[f'eef_{side}_rot'] = quat  # (w, x, y, z)
            entry[f'eef_{side}_pose'] = np.concatenate([pos, quat])

        entry['joint_pos_opensot'] = np.array([self.target_positions[n] for n in self.joint_names])
        entry['joint_pos_real'] = np.array([self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        entry['joint_vel_real'] = np.array([self.data.qvel[self.dof_adr[n]] for n in self.joint_names])

        if self.target_object_qpos_adr is not None:
            adr = self.target_object_qpos_adr
            entry['target_object_pose'] = self.data.qpos[adr:adr + 7].copy()  # (x,y,z, qw,qx,qy,qz)

        return entry

    def log_step(self):
        self.log_buffer.append(self.get_log_entry())

    def save_log(self, path: str):
        if not self.log_buffer:
            self.get_logger().warn("No log entries to save.")
            return
        keys = self.log_buffer[0].keys()
        stacked = {k: np.stack([entry[k] for entry in self.log_buffer]) for k in keys}
        np.savez(path, **stacked)
        self.get_logger().info(f"Saved {len(self.log_buffer)} log entries to {path}")

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = array.array('d', [self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        msg.velocity = array.array('d', [self.data.qvel[self.dof_adr[n]] for n in self.joint_names])
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TiagoProMujocoBridge()

    viewer_ctx = mujoco.viewer.launch_passive(node.model, node.data) if node.use_viewer else None

    try:
        while rclpy.ok() and (viewer_ctx is None or viewer_ctx.is_running()):
            start = time.perf_counter()

            if viewer_ctx is not None:
                with viewer_ctx.lock():
                    node.apply_targets()
                    node.actuate_grippers()
                    node.step_physics()
                viewer_ctx.sync()
            else:
                node.apply_targets()
                node.actuate_grippers()
                node.step_physics()

            node.publish_joint_state()
            node.log_step()
            rclpy.spin_once(node, timeout_sec=0)

            elapsed = time.perf_counter() - start
            sleep_time = (1.0 / node.fps) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        log_path = node.log_path or f"/tmp/tiago_pro_mujoco_log_{int(time.time())}.npz"
        node.save_log(log_path)
        if viewer_ctx is not None:
            viewer_ctx.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
