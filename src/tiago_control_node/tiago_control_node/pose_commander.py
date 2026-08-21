#!/usr/bin/env python3
"""Waypoint plan executor for the Tiago WBC end effector(s).

Define PLAN below as a sequence of waypoints and run this script - it drives
the arm(s) through them in order via /cartesian_interface/{side}/target_pose,
the same topics tiago_pro_opensot_node / tiago_opensot_node listen on. Those
nodes use the pose numbers directly as the desired end-effector pose in the
arm task's base frame (base_link by default), no TF lookup involved.

Usage:
  ros2 run tiago_control_node pose_commander
Gripper open/close commands go straight to the MuJoCo bridge on
/mujoco_bridge/gripper_{side}/open (Bool: True=open, False=close) - they only
take effect in simulation, there's no equivalent hardware topic here.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


# --- DEFINE YOUR PLAN HERE ---
# Each waypoint moves whichever side(s) are present, simultaneously, then
# holds for 'hold' seconds before moving to the next one. Orientation can be
# given as 'rpy_deg' (roll, pitch, yaw in degrees) or 'quat' (x, y, z, w).
# A 'gripper' key commands the MuJoCo bridge directly, e.g. {'right': 'close'}.
#
# Pick-and-place demo: right arm picks the cube up from (0.6, 0.0) on the table
# and places it at a different spot, (0.45, -0.25). quat [1,0,0,0] is a
# top-down approach (gripper pointing straight down at the table).
# These xyz/quat values were derived and reachability-checked against the
# robots/pal_tiago_pro MuJoCo model via numerical IK (see conversation history
# for the derivation) - but pose_commander targets the real solver's
# gripper_right_grasping_link frame from the actual URDF, whose exact offset
# from the wrist may not perfectly match the MuJoCo model used to derive
# these numbers. Treat as a verified-reachable starting point; nudge xyz by
# a centimeter or two if the real grasp misses.
TOP_DOWN = [1.0, 0.0, 0.0, 0.0]
FOURTY_FIVE_DEG_DOWN = R.from_euler('y', 45, degrees=True).as_quat().tolist()
PLAN = [
    {'hold': 4.0, 'right': {'xyz': [0.6, 0.0, 0.55], 'quat': TOP_DOWN}},           # pre-grasp, above cube
    {'hold': 3.0, 'right': {'xyz': [0.6, 0.0, 0.470], 'quat': FOURTY_FIVE_DEG_DOWN}},          # descend to cube
    {'hold': 2.0, 'gripper': {'right': 'close'}},                                  # grasp
    {'hold': 3.0, 'right': {'xyz': [0.6, 0.0, 0.55], 'quat': TOP_DOWN}},           # lift
    {'hold': 4.0, 'right': {'xyz': [0.45, -0.25, 0.55], 'quat': TOP_DOWN}},        # transport
    {'hold': 3.0, 'right': {'xyz': [0.45, -0.25, 0.470], 'quat': TOP_DOWN}},       # lower to place
    {'hold': 2.0, 'gripper': {'right': 'open'}},                                   # release
    {'hold': 3.0, 'right': {'xyz': [0.45, -0.25, 0.55], 'quat': TOP_DOWN}},        # retreat
]

class PoseCommander(Node):
    def __init__(self):
        super().__init__('pose_commander')

        self.declare_parameter('base_frame', 'opensot/base_link')
        self.declare_parameter('publish_rate', 30.0)
        self.frame_id = self.get_parameter('base_frame').value
        rate = self.get_parameter('publish_rate').value

        self.pubs = {
            'right': self.create_publisher(PoseStamped, '/cartesian_interface/right/target_pose', 10),
            'left': self.create_publisher(PoseStamped, '/cartesian_interface/left/target_pose', 10),
        }
        self.gripper_pubs = {
            'right': self.create_publisher(Bool, '/mujoco_bridge/gripper_right/open', 10),
            'left': self.create_publisher(Bool, '/mujoco_bridge/gripper_left/open', 10),
        }
        self.targets = {'right': None, 'left': None}
        self.gripper_targets = {'right': None, 'left': None}
        self.create_timer(1.0 / rate, self._publish_targets)

    def set_target(self, side, xyz, quat_xyzw):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in xyz)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in quat_xyzw)
        self.targets[side] = msg

    def set_gripper(self, side, status):
        self.gripper_targets[side] = (status == 'open')

    def _publish_targets(self):
        now = self.get_clock().now().to_msg()
        for side, msg in self.targets.items():
            if msg is not None:
                msg.header.stamp = now
                self.pubs[side].publish(msg)
        for side, is_open in self.gripper_targets.items():
            if is_open is not None:
                self.gripper_pubs[side].publish(Bool(data=is_open))

    def _spin_for(self, duration_sec):
        end_time = self.get_clock().now().nanoseconds / 1e9 + duration_sec
        while rclpy.ok() and self.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_plan(self, plan):
        for i, waypoint in enumerate(plan):
            hold = waypoint.get('hold', 3.0)
            for side in ('right', 'left'):
                if side not in waypoint:
                    continue
                xyz, quat = _resolve_pose(waypoint[side])
                self.set_target(side, xyz, quat)
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [{side}]: xyz={xyz}")

            for side, status in waypoint.get('gripper', {}).items():
                self.set_gripper(side, status)
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [gripper {side}]: {status}")

            self._spin_for(hold)

        self.get_logger().info("Plan complete - holding final waypoint. Ctrl+C to stop.")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def _resolve_pose(spec):
    xyz = spec['xyz']
    if 'quat' in spec:
        quat = spec['quat']
    else:
        quat = R.from_euler('xyz', spec.get('rpy_deg', [0, 0, 0]), degrees=True).as_quat().tolist()
    return xyz, quat


def main(args=None):
    rclpy.init(args=args)
    node = PoseCommander()

    try:
        node.run_plan(PLAN)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
