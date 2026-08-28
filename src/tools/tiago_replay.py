#!/usr/bin/env python3
"""Actions-only, open-loop replay of one recorded episode.

Answers a narrower question than a full policy eval: given the SAME starting condition a
demo was collected from, does the OpenSoT/MuJoCo pipeline alone - no policy, driven purely
by the demo's own recorded `actions` - reproduce the recorded outcome? If it doesn't, no
policy trained on this data can be expected to work either: the problem is upstream of the
policy (the dataset itself, the publishing/timing of actions during collection, or the
open-loop OpenSoT<->MuJoCo communication), not model quality.

This is deliberately NOT the --ground-truth mode mentioned elsewhere (README.md,
mujoco_sim_node.py's /mujoco_bridge/sim/set_joint_state) - that teleports MuJoCo's joint
state from the recording every step, which is a visual "did we record what we think we
recorded" check with no controller in the loop. This script is the opposite: it seeds the
sim ONCE, then leaves OpenSoT/MuJoCo running closed-loop against nothing but the recorded
cartesian actions, exactly like a deployed policy would drive them - a policy only ever gets
to command /cartesian_interface/{side}/target_pose + gripper opens, never to teleport a
joint straight to a recorded value. Seeding this replay with the recorded joint_pos_real by
default would hand it information a real policy never has and make a pass/fail meaningless -
so by default only the two things that could plausibly need restoring are: the fixed home
config every episode already starts from anyway (reset_robot_home already gives you this)
and the object's per-episode RANDOMIZED position (which genuinely needs restoring, or the
demo's absolute grasp waypoints point at empty table). --teleport-joint-state is kept as an
explicit opt-in escape hatch for debugging only, off by default.

Usage (bringup.launch.py and mujoco_bridge.launch.py must already be running), from
/home/forest_ws inside the container:
  python3 src/tools/tiago_replay.py --dataset /tmp/tiago_pro_episodes/dataset.h5 --demo 0
"""
import argparse
import time

import h5py
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from tiago_pro_mujoco_bridge.mujoco_sim_node import JOINT_TO_MOTOR
from tiago_control_node.tasks.pick_place_basket import check_success

RESET_TIMEOUT_SEC = 5.0
SETTLE_TIMEOUT_SEC = 5.0


class ActionReplay(Node):
    def __init__(self, base_frame: str):
        super().__init__('tiago_replay')
        self.base_frame = base_frame

        self.target_pubs = {
            'right': self.create_publisher(PoseStamped, '/cartesian_interface/right/target_pose', 10),
            'left': self.create_publisher(PoseStamped, '/cartesian_interface/left/target_pose', 10),
        }
        self.gripper_pubs = {
            'right': self.create_publisher(Bool, '/mujoco_bridge/gripper_right/open', 10),
            'left': self.create_publisher(Bool, '/mujoco_bridge/gripper_left/open', 10),
        }
        self.set_object_pose_pub = self.create_publisher(PoseStamped, '/mujoco_bridge/sim/set_object_pose', 10)
        self.set_joint_state_pub = self.create_publisher(JointState, '/mujoco_bridge/sim/set_joint_state', 10)
        self.reset_config_pub = self.create_publisher(Bool, '/streamdeck/reset_config', 10)
        self.reset_robot_home_cli = self.create_client(Trigger, '/mujoco_bridge/sim/reset_robot_home')

        self._reset_complete = False
        self.create_subscription(Bool, '/opensot/reset_complete', self._reset_complete_cb, 10)
        self._settled_at_home = False
        self.create_subscription(Bool, '/mujoco_bridge/settled_at_home', self._settled_at_home_cb, 10)

        self.object_xyz = None
        self.create_subscription(
            PoseStamped, '/mujoco_bridge/target_object_pose', self._object_pose_cb, 10)

    def _reset_complete_cb(self, msg: Bool):
        if msg.data:
            self._reset_complete = True

    def _settled_at_home_cb(self, msg: Bool):
        self._settled_at_home = msg.data

    def _object_pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.object_xyz = np.array([p.x, p.y, p.z])

    def _wait_for(self, predicate, timeout_sec, what):
        end_time = time.time() + timeout_sec
        while rclpy.ok() and not predicate() and time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not predicate():
            self.get_logger().warn(f"Timed out waiting for {what} after {timeout_sec}s - proceeding anyway.")

    def seed_episode_start(self, object_pose_row, joint_pos_row, teleport_joint_state: bool):
        """Reproduces the episode's starting condition the same way
        episode_orchestrator_node.py sequences a real reset - home reset -> (optional debug
        joint teleport) -> OpenSoT resync -> confirm settled -> only THEN move the object.
        That ordering (not "set object pose immediately after reset") matters: moving the
        object onto the table before the robot is confirmed settled risks a stale pre-reset
        command sweeping the arm through it (see mujoco_sim_node.py's own comments on
        exactly this hazard for the normal episode-reset path)."""
        self.get_logger().info("Waiting for /mujoco_bridge/sim/reset_robot_home service...")
        while not self.reset_robot_home_cli.wait_for_service(timeout_sec=2.0):
            pass
        future = self.reset_robot_home_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        if teleport_joint_state:
            msg = JointState()
            msg.name = list(JOINT_TO_MOTOR.keys())
            msg.position = [float(p) for p in joint_pos_row]
            self.set_joint_state_pub.publish(msg)
            self.get_logger().info(
                "--teleport-joint-state: teleported joint state to the episode's recorded "
                "start (debug only - a real policy never gets this).")

        self._reset_complete = False
        self.reset_config_pub.publish(Bool(data=True))
        self._wait_for(lambda: self._reset_complete, RESET_TIMEOUT_SEC, "/opensot/reset_complete")

        self._settled_at_home = False
        self._wait_for(lambda: self._settled_at_home, SETTLE_TIMEOUT_SEC, "/mujoco_bridge/settled_at_home")

        x, y, z, qx, qy, qz, qw = (float(v) for v in object_pose_row)
        obj_msg = PoseStamped()
        obj_msg.header.frame_id = self.base_frame
        obj_msg.pose.position.x, obj_msg.pose.position.y, obj_msg.pose.position.z = x, y, z
        (obj_msg.pose.orientation.x, obj_msg.pose.orientation.y,
         obj_msg.pose.orientation.z, obj_msg.pose.orientation.w) = qx, qy, qz, qw
        self.set_object_pose_pub.publish(obj_msg)
        self.get_logger().info(f"Object seeded to recorded start position ({x:.3f}, {y:.3f}, {z:.3f}).")

    def _publish_target(self, side, xyz, quat_xyzw):
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in xyz)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in quat_xyzw)
        self.target_pubs[side].publish(msg)

    def _publish_gripper(self, side, value):
        # value >= 0.5 means CLOSED - matches cartesian_interface_node._gripper_cb and
        # mujoco_sim_node.get_log_entry(), which together define this convention.
        self.gripper_pubs[side].publish(Bool(data=bool((value < 0.5))))

    def replay(self, actions: np.ndarray, dt: float):
        """Publishes one recorded action row per step, at the recorded cadence - raw,
        unsplined open-loop signal, exactly the rate/shape a policy's own action stream
        would look like at inference. Row layout: right_pos(3), right_quat_xyzw(4),
        left_pos(3), left_quat_xyzw(4), right_gripper(1), left_gripper(1) - confirmed exact
        in mujoco_sim_node.get_log_entry()."""
        n_steps = actions.shape[0]
        self.get_logger().info(f"Replaying {n_steps} steps at dt={dt:.4f}s (~{1.0/dt:.1f}Hz)...")
        for i in range(n_steps):
            row = actions[i]
            self._publish_target('right', row[0:3], row[3:7])
            self._publish_target('left', row[7:10], row[10:14])
            self._publish_gripper('right', row[14])
            self._publish_gripper('left', row[15])

            step_start = time.perf_counter()
            while rclpy.ok():
                remaining = dt - (time.perf_counter() - step_start)
                if remaining <= 0:
                    break
                rclpy.spin_once(self, timeout_sec=remaining)
        self.get_logger().info("Replay finished.")


def _load_demo(dataset_path: str, demo: str):
    demo_name = f'demo_{demo}' if demo.isdigit() else demo
    with h5py.File(dataset_path, 'r') as f:
        grp = f['data'][demo_name]
        actions = grp['actions'][:]
        object_pose_start = grp['obs']['target_object_pose'][0]
        object_pose_final_recorded = grp['obs']['target_object_pose'][-1, :3].astype(np.float64)
        joint_pos_start = grp['obs']['joint_pos_real'][0]
        fps = float(grp.attrs['fps'])
        recorded_success = bool(grp.attrs['success'])
    return demo_name, actions, object_pose_start, joint_pos_start, object_pose_final_recorded, fps, recorded_success


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dataset', required=True, help='Path to the collected .h5 dataset.')
    parser.add_argument('--demo', default='0', help='Episode to replay: a bare index (0) or a full group name (demo_3).')
    parser.add_argument('--base-frame', default='opensot/base_link',
                         help='Frame target poses are stamped with - must match pose_commander.py/mujoco_sim_node.py.')
    parser.add_argument('--rate-scale', type=float, default=1.0,
                         help='Multiplier on the recorded playback rate (>1 = faster, <1 = slower).')
    parser.add_argument('--teleport-joint-state', action='store_true',
                         help='Debug only: also teleport MuJoCo joints to the episode\'s recorded start '
                              '(a real policy never gets this - off by default, see module docstring).')
    args, ros_args = parser.parse_known_args()

    demo_name, actions, object_pose_start, joint_pos_start, object_pose_final_recorded, fps, recorded_success = \
        _load_demo(args.dataset, args.demo)
    dt = 1.0 / (fps * args.rate_scale)

    rclpy.init(args=ros_args)
    node = ActionReplay(args.base_frame)
    try:
        node.get_logger().info(
            f"Replaying {demo_name} from {args.dataset} ({actions.shape[0]} steps, "
            f"recorded success={recorded_success}, recorded fps={fps:.2f})")
        node.seed_episode_start(object_pose_start, joint_pos_start, args.teleport_joint_state)
        node.replay(actions, dt)

        # Let the final action's motion settle before reading the outcome.
        settle_end = time.perf_counter() + 1.0
        while rclpy.ok() and time.perf_counter() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.object_xyz is None:
            node.get_logger().error(
                "Never received /mujoco_bridge/target_object_pose - is mujoco_bridge.launch.py running?")
        else:
            success, message = check_success(node.object_xyz, pick_xyz=None)
            divergence = float(np.linalg.norm(node.object_xyz - object_pose_final_recorded))
            node.get_logger().info(f"Open-loop replay result: {message}")
            node.get_logger().info(
                f"Divergence from recorded final object position: {divergence:.4f}m "
                f"(replay={np.round(node.object_xyz, 3).tolist()}, "
                f"recorded={np.round(object_pose_final_recorded, 3).tolist()}) - "
                "large divergence here points at the collection/execution pipeline, not a policy.")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
