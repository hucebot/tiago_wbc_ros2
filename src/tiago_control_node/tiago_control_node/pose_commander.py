#!/usr/bin/env python3
"""Waypoint plan executor for the Tiago WBC end effector(s).

Define PLAN below as a sequence of waypoints and run this script - it drives
the arm(s) through them in order via /cartesian_interface/{side}/target_pose,
the same topics tiago_pro_opensot_node / tiago_opensot_node listen on. Those
nodes use the pose numbers directly as the desired end-effector pose in the
arm task's base frame (base_link by default), no TF lookup involved.

Waypoints are specified relative to the target object's pose, read from
/mujoco_bridge/target_object_pose (published by the MuJoCo bridge from the
live sim, see tiago_pro_mujoco_bridge/mujoco_bridge_node.py). This node waits
for that topic before running the plan, so the grasp/place trajectory tracks
wherever the object actually is instead of assuming a fixed spawn point -
required for randomizing the object pose across data-collection episodes.

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
# Position is given as 'xyz_rel', an [dx, dy, dz] offset added to the target
# object's xyz (captured once, at plan start) - or as 'xyz', an absolute
# base_link-frame position, for waypoints that aren't object-relative (e.g.
# the basket: it's a fixed static fixture, not the tracked object, so its
# waypoint uses its known world position directly instead of an offset).
#
# Pick-and-place-in-basket demo: right arm picks the object up off the table
# and drops it into the basket fixture (robots/pal_tiago_pro/xmls/scene_tiago_pro.xml,
# body "basket" - BASKET_XY/BASKET_HOVER_Z below must match that body's geometry).
# quat [1,0,0,0] is a top-down approach (gripper pointing straight down).
#
# APPROACH_Z_OFFSET/GRASP_Z_OFFSET (pick side) are tuned/verified-reachable
# against the robots/pal_tiago_pro MuJoCo model - pose_commander targets the real
# solver's gripper_right_grasping_link frame from the actual URDF, whose exact
# offset from the wrist may not perfectly match the MuJoCo model, so nudge by a
# centimeter or two if the real grasp misses. The grasp orientation ignores the
# object's own rotation (fine for a symmetric cube; revisit if the object changes).
#
# Table strikes were happening because pregrasp/descend/lift each used a different
# xy (and the very first move went straight from home - which isn't top-down - to a
# low, angled pregrasp), so the arm was simultaneously reorienting, translating
# laterally, AND dropping close to the table in one motion, with no guarantee
# OpenSoT's IK path stays clear of it along the way. Fixed by decoupling those three
# things: GRASP_XY_OFFSET is now shared by every pick-phase waypoint (pregrasp,
# descend, lift, transit) so the only thing that ever changes near the table is
# height, one axis at a time; TRANSIT_Z_OFFSET adds a safe waypoint well above the
# table where the big home -> top-down reorientation and any lateral travel happen,
# clear of any collision risk.
#
# The basket has 5cm walls with the floor's top at 0.48 and the rim at 0.53
# (see the XML); BASKET_HOVER_Z hovers 3cm above the rim and releases there
# rather than descending inside the walls, to keep the gripper clear of them -
# the object free-falls the last few cm into the basket. Watch the first few
# drops in the viewer and tighten BASKET_HOVER_Z if it's bouncing out.
TOP_DOWN = [1.0, 0.0, 0.0, 0.0]
FOURTY_FIVE_DEG_DOWN = R.from_euler('y', 90, degrees=True).as_quat().tolist()
GRASP_XY_OFFSET = [0.00, 0.0]  # lateral nudge from the object's own xy to the actual grasp point -
                                # shared by every pick-phase waypoint, see note above
TRANSIT_Z_OFFSET = 0.08    # safe height above the object for reorienting/traveling laterally
APPROACH_Z_OFFSET = 0.023  # above the object: pre-grasp / lift height, close to the table
GRASP_Z_OFFSET = -0.057    # at the object: descend-to-grasp height
BASKET_XY = (0.78, 0.0)    # must match the "basket" body's pos in scene_tiago_pro.xml
BASKET_HOVER_Z = 0.56      # basket rim (0.53) + 3cm clearance
PLAN = [
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': FOURTY_FIVE_DEG_DOWN}},     # transit height above the grasp point - reorients to top-down here, clear of the table
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, APPROACH_Z_OFFSET], 'quat': FOURTY_FIVE_DEG_DOWN}},    # pre-grasp - straight down from transit, same xy/orientation
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, GRASP_Z_OFFSET], 'quat': FOURTY_FIVE_DEG_DOWN}},       # descend to object - straight down, no lateral motion
    {'hold': 1.0, 'gripper': {'right': 'close'}},                                                     # grasp
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, APPROACH_Z_OFFSET], 'quat': FOURTY_FIVE_DEG_DOWN}},    # lift - straight up, same xy as grasp
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': FOURTY_FIVE_DEG_DOWN}},     # lift further to transit height - straight up
    {'hold': 2.0, 'right': {'xyz': [*BASKET_XY, BASKET_HOVER_Z], 'quat': FOURTY_FIVE_DEG_DOWN}},                 # transport, hover over the basket - lateral move, already at a safe height
    {'hold': 2.0, 'gripper': {'right': 'open'}},                                                       # release - drops into the basket
    {'hold': 1.0, 'right': {'xyz_rel': [*GRASP_XY_OFFSET, TRANSIT_Z_OFFSET], 'quat': TOP_DOWN}},     # retreat to transit height, back over the pick spot
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
        self.object_xyz = None
        self.create_subscription(
            PoseStamped, '/mujoco_bridge/target_object_pose', self._object_pose_cb, 10)
        self.create_timer(1.0 / rate, self._publish_targets)

    def _object_pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.object_xyz = (p.x, p.y, p.z)

    def wait_for_object_pose(self, timeout_sec=10.0):
        self.get_logger().info("Waiting for target object pose on /mujoco_bridge/target_object_pose...")
        end_time = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok() and self.object_xyz is None and self.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.object_xyz is None:
            raise RuntimeError(
                "Timed out waiting for /mujoco_bridge/target_object_pose - is the MuJoCo bridge running "
                "with a valid target_object_joint?")
        self.get_logger().info(f"Target object at {self.object_xyz}")

    def set_target(self, side, xyz, quat_xyzw):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in xyz)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = (float(v) for v in quat_xyzw)
        self.targets[side] = msg

    def set_gripper(self, side, status):
        self.gripper_targets[side] = (status == 'open')

    def clear_targets(self):
        """Stops the publish timer from re-sending the last-held waypoint. Call this
        after an external reset (e.g. /mujoco_bridge/end_episode) - otherwise, as long
        as this node/process is still alive, it keeps republishing the previous plan's
        final pose and immediately fights the reset."""
        self.targets = {'right': None, 'left': None}
        self.gripper_targets = {'right': None, 'left': None}

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
        """Runs one pass of `plan` and returns the object xyz it was resolved against.

        That object pose is captured once, before the first waypoint, and reused for
        every 'xyz_rel' waypoint in this call - self.object_xyz itself keeps updating
        live off the /mujoco_bridge/target_object_pose subscription (including while
        the object is mid-air in the gripper), so reading it fresh per-waypoint would
        anchor later waypoints (transport/place) to the object's current in-hand
        position instead of where it started.
        """
        self.wait_for_object_pose()
        object_xyz = self.object_xyz
        for i, waypoint in enumerate(plan):
            hold = waypoint.get('hold', 3.0)
            for side in ('right', 'left'):
                if side not in waypoint:
                    continue
                xyz, quat = _resolve_pose(waypoint[side], object_xyz)
                self.set_target(side, xyz, quat)
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [{side}]: xyz={xyz}")

            for side, status in waypoint.get('gripper', {}).items():
                self.set_gripper(side, status)
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [gripper {side}]: {status}")

            self._spin_for(hold)

        return object_xyz


def _resolve_pose(spec, object_xyz):
    if 'xyz_rel' in spec:
        dx, dy, dz = spec['xyz_rel']
        ox, oy, oz = object_xyz
        xyz = [ox + dx, oy + dy, oz + dz]
    else:
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
        node.get_logger().info("Plan complete - holding final waypoint. Ctrl+C to stop.")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
