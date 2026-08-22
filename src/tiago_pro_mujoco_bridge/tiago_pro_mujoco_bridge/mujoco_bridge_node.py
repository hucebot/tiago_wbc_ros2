#!/usr/bin/env python3
"""MuJoCo-based simulated hardware bridge for Tiago Pro.

Stands in for the real robot on the tiago_pro_opensot_node / cartesian_interface_node
pipeline: publishes /joint_states from a live MuJoCo simulation and drives the
simulated torso + arm actuators from the OpenSoT solver's /opensot/joint_states output.

The tiago-pro-mujoco XML is fixed-base and headless (no wheel or head joints), so
base and head commands from OpenSoT are simply not applied here.
"""
import os
import time
import array

import numpy as np
import h5py
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from tiago_control_node.utils import tiago_pro_home_config

# Joint name -> MuJoCo position actuator name (only the DoFs this MuJoCo model actuates)
JOINT_TO_MOTOR = {
    'torso_lift_joint': 'torso_lift_motor',
    **{f'arm_left_{i}_joint': f'arm_left_{i}_motor' for i in range(1, 8)},
    **{f'arm_right_{i}_joint': f'arm_right_{i}_motor' for i in range(1, 8)},
}

# Safety net if /opensot/reset_complete never arrives (e.g. tiago_pro_opensot_node isn't
# running or crashed mid-reset) - without this, logging would stay silently paused forever.
MAX_RESET_WAIT_SEC = 5.0

# Measured empirically in MuJoCo: 0.0 rad -> ~102mm fingertip gap (open), 0.9 rad -> ~7mm gap
# (closed, unobstructed). This is the OPPOSITE of tiago_pro_sim.py's comment - verified
# directly against this XML's linkage.
#
# GRIPPER_CLOSED_POS used to just be that fully-closed 0.9 - fine for an empty gripper, but
# with a ~40mm cube between the fingers, the position servo (kp=5, actuatorfrcrange +-8 on
# the driven finger joints) kept commanding a gap 33mm smaller than what's physically there,
# so it pressed into the object for the entire "closed" duration instead of stopping at
# contact - which is what was actually causing the "slipping" (deep interpenetration, not a
# friction problem). Retargeted to ~34mm gap: some excess-close beyond the object's own width
# is still needed to generate real grip force (that's how a position-controlled parallel
# gripper squeezes at all), just not 33mm of it. The 102mm/7mm mapping is an empirical
# 2-point measurement, assumed roughly linear here to back out a angle for a specific gap -
# this hasn't been reverified against the real gap once a 40mm object is actually in the
# way, so nudge it in the viewer if the grip still looks too tight or too loose.
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSED_POS = 0.55

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
        self.declare_parameter('episode_log_path', '/tmp/tiago_pro_episodes/dataset.h5')
        self.declare_parameter('save_failed_episodes', False)
        # Table-frame xy range the object is respawned into on /mujoco_bridge/end_episode.
        # Kept small and centered on the cube's XML spawn point (0.6, -0.15) - the right
        # side of the table, in the right arm's workspace (see pose_commander.py's PLAN,
        # which then carries it to the basket on the table's far/left side); widen once
        # you've confirmed the corners are still reachable for your grasp approach.
        self.declare_parameter('object_x_range', [0.55, 0.65])
        self.declare_parameter('object_y_range', [-0.20, -0.10])
        # Frame the published target-object pose is expressed in. This MuJoCo scene is
        # fixed-base with the robot's base_link coincident with the MuJoCo world origin,
        # so raw world-frame qpos doubles as the base_link-frame pose the arm tasks expect
        # (see pose_commander.py, which consumes this topic directly as base_link-frame xyz).
        self.declare_parameter('base_frame', 'opensot/base_link')

        xml_path = self.get_parameter('mujoco_xml_path').value
        self.use_viewer = self.get_parameter('viewer').value
        self.fps = self.get_parameter('fps').value
        command_topic = self.get_parameter('command_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        target_object_joint = self.get_parameter('target_object_joint').value
        self.episode_log_path = self.get_parameter('episode_log_path').value
        self.save_failed_episodes = self.get_parameter('save_failed_episodes').value
        self.object_x_range = tuple(self.get_parameter('object_x_range').value)
        self.object_y_range = tuple(self.get_parameter('object_y_range').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.episode_idx = 0     # total reset attempts, success or not
        self.saved_episode_idx = 0  # counts only episodes actually written to disk - keeps
                                     # saved filenames contiguous even when failures are skipped

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
            self.target_object_dof_adr = None
        else:
            self.target_object_qpos_adr = self.model.jnt_qposadr[tjid]
            self.target_object_dof_adr = self.model.jnt_dofadr[tjid]

        self.log_buffer = []
        self.logging_paused = False
        self.logging_paused_since = None
        self.object_randomize_pending = False
        # "Action" for imitation learning, AND obs/eef_{side}_pose (see get_log_entry) = the
        # commanded /cartesian_interface/{side}/target_pose - confirmed against the real-robot
        # pipeline (Dont-Be-Brave/scripts/inference/tiago_ros2.py's get_synchronized_observation
        # sources eef_{side}_pose from this exact topic, and it's also used as the action
        # reconstruction anchor there - there's no separate "action" signal on the real robot).
        # Holds the latest received pose per side. This topic never expires on its own (a stale
        # value would sit here forever once received, even after the publisher exits), so it's
        # cleared on every reset (see _reset_to_home) - otherwise a new episode's first few
        # logged steps would show the previous episode's final commanded pose.
        self.latest_action = {'left': None, 'right': None}

        self._reset_to_home()
        # mj_resetData (inside _reset_to_home) put the object back at its XML-defined
        # spawn height, resting on the table - capture that once, here, as the known-good
        # z for every future respawn. _randomize_object_pose() runs standalone later (not
        # right after a full mj_resetData), so it can't re-derive this from current qpos -
        # by then the object could be anywhere (e.g. still settling in the basket).
        if self.target_object_qpos_adr is not None:
            self.object_rest_z = float(self.data.qpos[self.target_object_qpos_adr + 2])

        self.command_sub = self.create_subscription(
            JointState, command_topic, self._command_cb, 10)
        self.joint_state_pub = self.create_publisher(
            JointState, joint_states_topic, 10)
        self.target_object_pose_pub = self.create_publisher(
            PoseStamped, '/mujoco_bridge/target_object_pose', 10)
        self.reset_config_pub = self.create_publisher(Bool, '/streamdeck/reset_config', 10)
        # Fires once the *entire* reset (robot settled AND object respawned) is actually
        # done - /mujoco_bridge/end_episode's response only means "reset requested", since
        # the robot settle + object respawn both finish asynchronously afterward (see
        # _reset_complete_cb). Callers must wait for this before trusting target_object_pose
        # or issuing new commands, or they'll act on the stale pre-reset object position
        # while the arm is still mid-resync.
        self.episode_ready_pub = self.create_publisher(Bool, '/mujoco_bridge/episode_ready', 10)

        for side in ('left', 'right'):
            self.create_subscription(
                Bool, f'/mujoco_bridge/gripper_{side}/open',
                lambda msg, s=side: self._gripper_cmd_cb(s, msg), 10)
            self.create_subscription(
                PoseStamped, f'/cartesian_interface/{side}/target_pose',
                lambda msg, s=side: self._action_cb(s, msg), 10)

        # tiago_pro_opensot_node publishes this the instant it's actually finished
        # resyncing after a reset (see /streamdeck/reset_config below) - gating logging
        # on it, rather than a fixed delay, is exact regardless of how long that resync
        # happens to take.
        self.create_subscription(Bool, '/opensot/reset_complete', self._reset_complete_cb, 10)

        self.end_episode_srv = self.create_service(
            SetBool, '/mujoco_bridge/end_episode', self._end_episode_cb)

        self.get_logger().info(
            f"Tiago Pro MuJoCo bridge ready. Driving {len(self.joint_names)} joints "
            f"from '{command_topic}', publishing state on '{joint_states_topic}'.")

    def _gripper_cmd_cb(self, side, msg: Bool):
        self.gripper_status[side] = 'open' if msg.data else 'close'

    def _action_cb(self, side, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        self.latest_action[side] = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def _reset_complete_cb(self, msg: Bool):
        if msg.data:
            # tiago_pro_opensot_node has confirmed the arm is actually settled at home -
            # only now is it safe to respawn the object (see _end_episode_cb: resetting
            # the robot and randomizing the object in the same instant risked the newly
            # spawned object overlapping the not-yet-settled arm, which MuJoCo's contact
            # solver would resolve by violently flinging them apart on the next step).
            self._finish_reset()

    def _finish_reset(self):
        if self.object_randomize_pending:
            self._randomize_object_pose()
            self.object_randomize_pending = False
        self.logging_paused = False
        self.episode_ready_pub.publish(Bool(data=True))

    def _reset_to_home(self):
        mujoco.mj_resetData(self.model, self.data)
        for joint_name, pos in HOME_POSITIONS.items():
            if joint_name not in self.qpos_adr:
                continue
            self.data.qpos[self.qpos_adr[joint_name]] = pos
            self.data.ctrl[self.actuator_id[joint_name]] = pos
        for name, aid in self.gripper_actuator_id.items():
            self.data.ctrl[aid] = GRIPPER_OPEN_POS
        self.current_gripper_pos = {'left': GRIPPER_OPEN_POS, 'right': GRIPPER_OPEN_POS}
        self.gripper_status = {'left': 'open', 'right': 'open'}
        # apply_targets() overwrites data.ctrl from this dict every step - reset it too,
        # otherwise the very next control loop iteration immediately re-applies whatever
        # stale (non-home) joint targets OpenSoT last published, undoing the reset above.
        self.target_positions = dict(HOME_POSITIONS)
        # Same staleness problem for the logged action/obs pose: /cartesian_interface/{side}/
        # target_pose never expires on its own (see latest_action's declaration), so without
        # clearing this, the first few logged steps of a new episode would show the *previous*
        # episode's final commanded pose.
        self.latest_action = {'left': None, 'right': None}
        # mj_resetData above already put the object back at its safe default XML spawn
        # point (already known non-overlapping with the home posture) - leave it there
        # until the robot is confirmed settled; see _reset_complete_cb.
        mujoco.mj_forward(self.model, self.data)

    def _randomize_object_pose(self):
        if self.target_object_qpos_adr is None:
            return
        adr = self.target_object_qpos_adr
        x = float(np.random.uniform(*self.object_x_range))
        y = float(np.random.uniform(*self.object_y_range))
        self.data.qpos[adr:adr + 3] = [x, y, self.object_rest_z]
        self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]  # upright, no rotation
        if self.target_object_dof_adr is not None:
            self.data.qvel[self.target_object_dof_adr:self.target_object_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _end_episode_cb(self, request, response):
        """Finalizes the just-finished episode (saves its HDF5 log with the given
        success flag) and resets the sim - robot to home first, object respawned at a
        new random table position only once the robot is confirmed settled there (see
        _reset_complete_cb) - so the caller can then immediately start the next one."""
        saved_path = self._save_episode_log(success=request.data)
        self._reset_to_home()
        self.episode_idx += 1
        self.object_randomize_pending = True
        # Resetting MuJoCo's own qpos isn't enough on its own: tiago_pro_opensot_node
        # runs a separate IK integrator that caches whatever cartesian target it last
        # received on /cartesian_interface/{side}/target_pose *indefinitely* (nothing
        # ever expires it, even if the publisher has since exited) and keeps solving
        # toward it every control cycle - immediately fighting the reset above unless
        # told to let go. /streamdeck/reset_config is the existing hook for that: it
        # clears those cached targets and re-syncs the solver's joint state (which,
        # with no real ros2_control hardware feedback in this sim, falls back straight
        # to the same home_config both sides already agree on) - and its completion is
        # what _reset_complete_cb waits for before respawning the object.
        self.logging_paused = True
        self.logging_paused_since = time.time()
        self.reset_config_pub.publish(Bool(data=True))
        response.success = True
        response.message = saved_path or "no steps logged for this episode"
        return response

    def _save_episode_log(self, success: bool):
        if not self.log_buffer:
            return None
        if not success and not self.save_failed_episodes:
            self.get_logger().info(
                f"Episode {self.episode_idx} failed ({len(self.log_buffer)} steps) - discarding, "
                "not saved (save_failed_episodes:=true to keep failures too).")
            self.log_buffer = []
            return None
        os.makedirs(os.path.dirname(self.episode_log_path), exist_ok=True)
        demo_name = f'demo_{self.saved_episode_idx}'
        # Append mode: one persistent file across the whole collection run, one new
        # top-level group per episode. Opened/closed per episode (not held open for the
        # run's duration) so a crash mid-run can't corrupt already-saved demos.
        with h5py.File(self.episode_log_path, 'a') as f:
            # Dont-Be-Brave's Tiago task reads h5py.File(fpath, "r")["data"][demo_name] -
            # the top-level "data" group is required, not optional.
            data_grp = f.require_group('data')
            grp = data_grp.create_group(demo_name)
            grp.create_dataset('actions', data=np.stack([e['actions'] for e in self.log_buffer]))
            obs_grp = grp.create_group('obs')
            for k in self.log_buffer[0]['obs'].keys():
                obs_grp.create_dataset(k, data=np.stack([e['obs'][k] for e in self.log_buffer]))
            grp.attrs['success'] = bool(success)
            grp.attrs['num_steps'] = len(self.log_buffer)
            grp.attrs['attempt_index'] = self.episode_idx
        self.get_logger().info(
            f"Saved data/{demo_name} ({len(self.log_buffer)} steps, success={success}) to {self.episode_log_path}")
        self.saved_episode_idx += 1
        self.log_buffer = []
        return f'{self.episode_log_path}::data/{demo_name}'

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
        """Snapshots the current sim state for offline logging (camera streams excluded).

        Schema matches Dont-Be-Brave/timid's Tiago task exactly (see tasks/tiago.py's
        _split_actions and config/tiago_config.py's data_specs):
          - obs/eef_{side}_pose, obs/joint_pos_opensot, obs/joint_pos_real,
            obs/target_object_pose: all (x,y,z,qx,qy,qz,qw) where poses, ROS quaternion order.
          - actions: single flat (16,) vector - [right_pos(3), right_quat(4), left_pos(3),
            left_quat(4), right_gripper(1), left_gripper(1)] - per _split_actions' slicing.
        obs/eef_{side}_pos/_rot are MuJoCo's own physics ground truth (extra, sim-only,
        (w,x,y,z) MuJoCo quaternion order - NOT part of the trained schema, harmless/ignored
        by DataSpec.filter if unused, useful for our own debugging)."""
        obs = {}
        eef_pose_xyzw = {}
        for side in ('left', 'right'):
            sim_pose_xyzw = None
            sid = self.eef_site_id[side]
            if sid != -1:
                pos = self.data.site_xpos[sid].copy()
                quat_wxyz = np.zeros(4)
                mujoco.mju_mat2Quat(quat_wxyz, self.data.site_xmat[sid])
                obs[f'eef_{side}_pos'] = pos
                obs[f'eef_{side}_rot'] = quat_wxyz  # (w, x, y, z) - MuJoCo ground truth, sim-only
                sim_pose_xyzw = np.concatenate([pos, quat_wxyz[[1, 2, 3, 0]]])

            action = self.latest_action[side]
            if action is None:
                # No /cartesian_interface/{side}/target_pose message received yet this
                # episode (e.g. the left arm is never commanded in this task) - fall back to
                # MuJoCo's own current eef pose so this is still a valid, absolute EEF pose
                # rather than something a policy would have to special-case.
                action = sim_pose_xyzw if sim_pose_xyzw is not None else np.zeros(7)
            eef_pose_xyzw[side] = action
            obs[f'eef_{side}_pose'] = action

        obs['joint_pos_opensot'] = np.array([self.target_positions[n] for n in self.joint_names])
        obs['joint_pos_real'] = np.array([self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        obs['joint_vel_real'] = np.array([self.data.qvel[self.dof_adr[n]] for n in self.joint_names])

        if self.target_object_qpos_adr is not None:
            adr = self.target_object_qpos_adr
            x, y, z, qw, qx, qy, qz = self.data.qpos[adr:adr + 7]
            obs['target_object_pose'] = np.array([x, y, z, qx, qy, qz, qw])

        # 1.0 = closed, 0.0 = open - matches cartesian_interface_node._gripper_cb's threshold
        # (point.x > 0.5 -> CLOSED) and the inference script's _make_gripper_msg (val >= 0.5
        # -> point.x = 1.0), which together define what a "gripper" action value >=0.5 means
        # for both vive-teleop recording and replay on the real robot.
        right_gripper = 1.0 if self.gripper_status['right'] == 'close' else 0.0
        left_gripper = 1.0 if self.gripper_status['left'] == 'close' else 0.0
        actions = np.concatenate([
            eef_pose_xyzw['right'], eef_pose_xyzw['left'], [right_gripper], [left_gripper],
        ]).astype(np.float32)

        return {'actions': actions, 'obs': obs}

    def log_step(self):
        if self.logging_paused and time.time() - self.logging_paused_since > MAX_RESET_WAIT_SEC:
            self.get_logger().warn(
                f"No /opensot/reset_complete after {MAX_RESET_WAIT_SEC}s - is tiago_pro_opensot_node "
                "running? Resuming logging (and any pending object respawn) anyway so data/episodes "
                "aren't silently stuck.")
            self._finish_reset()
        if not self.logging_paused:
            self.log_buffer.append(self.get_log_entry())

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = array.array('d', [self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        msg.velocity = array.array('d', [self.data.qvel[self.dof_adr[n]] for n in self.joint_names])
        self.joint_state_pub.publish(msg)

    def publish_target_object_pose(self):
        if self.target_object_qpos_adr is None:
            return
        adr = self.target_object_qpos_adr
        x, y, z, qw, qx, qy, qz = self.data.qpos[adr:adr + 7]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = float(qx), float(qy), float(qz), float(qw)
        self.target_object_pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TiagoProMujocoBridge()

    viewer_ctx = mujoco.viewer.launch_passive(node.model, node.data) if node.use_viewer else None

    try:
        while rclpy.ok() and (viewer_ctx is None or viewer_ctx.is_running()):
            start = time.perf_counter()

            if viewer_ctx is not None:
                # spin_once is inside the lock too: /mujoco_bridge/end_episode's callback
                # writes data.qpos directly (via _reset_to_home), and the passive viewer
                # reads data from its own render thread - an unlocked reset would race it.
                with viewer_ctx.lock():
                    node.apply_targets()
                    node.actuate_grippers()
                    node.step_physics()
                    rclpy.spin_once(node, timeout_sec=0)
                viewer_ctx.sync()
            else:
                node.apply_targets()
                node.actuate_grippers()
                node.step_physics()
                rclpy.spin_once(node, timeout_sec=0)

            node.publish_joint_state()
            node.publish_target_object_pose()
            node.log_step()

            elapsed = time.perf_counter() - start
            sleep_time = (1.0 / node.fps) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        node._save_episode_log(success=False)  # outcome unknown at shutdown/Ctrl+C
        if viewer_ctx is not None:
            viewer_ctx.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
