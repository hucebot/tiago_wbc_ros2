#!/usr/bin/env python3
"""Generic waypoint plan executor for the Tiago WBC end effector(s).

This file has no task-specific knowledge - it just knows how to run a PLAN (a
list of waypoints; see tiago_control_node/tasks/pick_place_basket.py for the
format and an example) by publishing to /cartesian_interface/{side}/target_pose,
the same topics tiago_pro_opensot_node / tiago_opensot_node listen on. Those
nodes use the pose numbers directly as the desired end-effector pose in the
arm task's base frame (base_link by default), no TF lookup involved.

Waypoints can be specified relative to the target object's pose, read from
/mujoco_bridge/target_object_pose (published by the MuJoCo bridge from the
live sim). This node waits for that topic before running a plan, so an
object-relative grasp/place trajectory tracks wherever the object actually is
instead of assuming a fixed spawn point - required for randomizing the object
pose across data-collection episodes.

To run a different task, edit the import in main() below to point at a
different tasks/*.py module.

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
from scipy.spatial.transform import Rotation as R, Slerp


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
        # (xyz, quat) of the last waypoint commanded per side, so the NEXT waypoint's
        # motion can be interpolated from here rather than jumped to - see _move_to().
        self._last_waypoint_pose = {'right': None, 'left': None}
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
        self._last_waypoint_pose = {'right': None, 'left': None}

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
            side_targets = {}
            for side in ('right', 'left'):
                if side not in waypoint:
                    continue
                xyz, quat = _resolve_pose(waypoint[side], object_xyz)
                side_targets[side] = (xyz, quat)
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [{side}]: xyz={xyz}")

            for side, status in waypoint.get('gripper', {}).items():
                self.set_gripper(side, status)
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [gripper {side}]: {status}")

            if side_targets:
                self._move_to(side_targets, hold)
            else:
                self._spin_for(hold)  # gripper-only waypoint - nothing to move

        return object_xyz

    def _move_to(self, side_targets, duration_sec):
        """Publishes a continuously-interpolated target from each side's last commanded
        pose to its new one over duration_sec, instead of jumping straight there and
        holding a fixed value for the whole duration.

        This matters beyond just "looking smoother": the commanded pose here is exactly
        what gets logged as actions/eef_{side}_pose (see mujoco_sim_node.py's
        get_log_entry - it mirrors /cartesian_interface/{side}/target_pose, not ground
        truth motion), and that's also true on the real robot, where it's driven by a
        continuously-tracked Vive controller. Jumping-and-holding here would make a
        scripted plan's recorded actions look like a handful of discrete steps instead of
        the continuously-varying signal a human teleoperator produces - a training-data
        mismatch, not just a visual one.
        """
        starts = {side: self._last_waypoint_pose[side] or pose for side, pose in side_targets.items()}
        rotations = {
            side: Slerp([0.0, 1.0], R.from_quat([starts[side][1], target_quat]))
            for side, (_, target_quat) in side_targets.items()
        }

        start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            frac = 1.0 if duration_sec <= 0 else min(
                1.0, (self.get_clock().now().nanoseconds / 1e9 - start_time) / duration_sec)
            for side, (target_xyz, _) in side_targets.items():
                start_xyz, _ = starts[side]
                xyz = [(1 - frac) * s + frac * t for s, t in zip(start_xyz, target_xyz)]
                quat = rotations[side](frac).as_quat().tolist()
                self.set_target(side, xyz, quat)
            rclpy.spin_once(self, timeout_sec=0.02)
            if frac >= 1.0:
                break

        self._last_waypoint_pose.update(side_targets)


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
    # Change this import to run a different task.
    from tiago_control_node.tasks.pick_place_basket import PLAN

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
