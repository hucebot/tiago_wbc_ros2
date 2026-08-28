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
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import PchipInterpolator
from scipy.spatial.transform import Rotation as R, RotationSpline
from tf2_ros import Buffer, TransformListener, TransformException


class PoseCommander(Node):
    def __init__(self):
        super().__init__('pose_commander')

        self.declare_parameter('base_frame', 'opensot/base_link')
        self.declare_parameter('publish_rate', 30.0)
        # Same param names/defaults as tiago_pro_opensot_node.py's frames.*_gripper - these
        # need to name the SAME links in the SAME (opensot ghost) TF tree that node solves
        # for, since _get_current_pose() below reads it as "where OpenSoT currently has the
        # arm", which is exactly what a target published straight to
        # /cartesian_interface/{side}/target_pose is asking it to move away from.
        self.declare_parameter('frames.right_gripper', 'gripper_right_grasping_link')
        self.declare_parameter('frames.left_gripper', 'gripper_left_grasping_link')
        self.frame_id = self.get_parameter('base_frame').value
        self.gripper_frames = {
            'right': self.get_parameter('frames.right_gripper').value,
            'left': self.get_parameter('frames.left_gripper').value,
        }
        rate = self.get_parameter('publish_rate').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        # (xyz, quat) of the last waypoint commanded per side, so the NEXT run_plan() call's
        # spline can start from here rather than jumping to it - see _execute_spline(). None
        # until a plan has actually moved that side at least once (or after clear_targets());
        # run_plan() resolves a None here via a live TF lookup (_get_current_pose) instead of
        # assuming the arm is already at the first waypoint, so the very first segment of a
        # session/episode is a real spline from the arm's true current pose too, not a step.
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

    def _osot(self, frame_id):
        """Prefixes a plain frame name into OpenSoT's ghost TF tree - mirrors
        cartesian_interface_node.py's identical helper. Idempotent (strips any existing
        'opensot/' first) so it's safe to call on a frame that's already prefixed."""
        clean = frame_id.replace('opensot/', '').lstrip('/')
        return f'opensot/{clean}'

    def _get_current_pose(self, side, timeout_sec=5.0):
        """Live FK of the OpenSoT ghost's current gripper pose (base_frame -> gripper frame,
        both in the opensot/ tree) - used by run_plan() to seed the spline's start point
        when _last_waypoint_pose[side] is None, instead of assuming the arm is already at
        the first waypoint (see that field's comment for why that assumption was wrong)."""
        base_frame = self._osot(self.frame_id)
        target_frame = self._osot(self.gripper_frames[side])
        end_time = self.get_clock().now().nanoseconds / 1e9 + timeout_sec
        while rclpy.ok():
            try:
                t = self.tf_buffer.lookup_transform(base_frame, target_frame, rclpy.time.Time())
                pos, rot = t.transform.translation, t.transform.rotation
                return [pos.x, pos.y, pos.z], [rot.x, rot.y, rot.z, rot.w]
            except TransformException as exc:
                if self.get_clock().now().nanoseconds / 1e9 > end_time:
                    raise RuntimeError(
                        f"Timed out waiting for TF {base_frame} -> {target_frame} - is "
                        f"tiago_pro_opensot_node running? ({exc})"
                    ) from exc
                rclpy.spin_once(self, timeout_sec=0.1)

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

    def run_plan(self, plan):
        """Runs one pass of `plan` and returns the object xyz it was resolved against.

        That object pose is captured once, before the first waypoint, and reused for
        every 'xyz_rel' waypoint in this call - self.object_xyz itself keeps updating
        live off the /mujoco_bridge/target_object_pose subscription (including while
        the object is mid-air in the gripper), so reading it fresh per-waypoint would
        anchor later waypoints (transport/place) to the object's current in-hand
        position instead of where it started.

        Builds ONE timeline for the whole plan (each waypoint's 'hold' becomes the time
        it's reached at, not a pause after arriving) and hands it to _execute_spline() as
        a single continuous motion, rather than running each waypoint as its own separate
        linear interpolation. A per-waypoint LERP is only C0 continuous - velocity jumps
        instantaneously at every waypoint boundary, and a gripper-only waypoint was a dead
        stop (_spin_for) with a sudden restart after it. Splining the whole sequence at
        once gives continuous velocity throughout, including through gripper-only holds
        (see the "carry the last real target forward" comment below).
        """
        self.wait_for_object_pose()
        object_xyz = self.object_xyz

        # One knot list per side: (time, xyz, quat) tuples the spline must pass through.
        knots = {'right': [], 'left': []}
        gripper_events = []  # (time, side, status), fired during _execute_spline as t crosses them
        last_target = dict(self._last_waypoint_pose)
        t = 0.0

        sides_used = {side for waypoint in plan for side in ('right', 'left') if side in waypoint}
        for side in sides_used:
            if last_target[side] is None:
                last_target[side] = self._get_current_pose(side)
                self.get_logger().info(
                    f"No prior commanded pose for {side} - starting this plan's spline from "
                    f"its live TF pose: xyz={last_target[side][0]}"
                )
        # last_target gets overwritten below as the plan's waypoints are walked - snapshot
        # the resolved starting pose now, before that happens, to hand to _execute_spline.
        start_poses = dict(last_target)

        for i, waypoint in enumerate(plan):
            hold = max(waypoint.get('hold', 3.0), 1e-3)  # keep knot times strictly increasing
            # Gripper events fire at the START of this waypoint's window (t, before hold is
            # added), not the end - matching the old _spin_for-based behavior (set the
            # gripper, THEN wait `hold` seconds for it to actually move) instead of the
            # spatial knots below, which are reached BY the end of their window. Firing at
            # t_end instead (as an earlier version of this did) put the LAST waypoint's
            # gripper event at exactly total_duration - the same instant _execute_spline's
            # loop exits - leaving zero remaining spin time for the 30Hz _publish_targets
            # timer to actually publish it before run_plan() returns. That's invisible in
            # pose_commander.py's standalone main() (which keeps spinning indefinitely
            # after run_plan() returns) but real in episode_manager.py's loop, where
            # run_plan() returning goes straight into clear_targets() for the next episode -
            # wiping gripper_targets back to None before the last command ever got sent.
            for side, status in waypoint.get('gripper', {}).items():
                gripper_events.append((t, side, status))
                self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [gripper {side}]: {status}")

            t += hold
            for side in ('right', 'left'):
                if side in waypoint:
                    xyz, quat = _resolve_pose(waypoint[side], object_xyz)
                    last_target[side] = (xyz, quat)
                    knots[side].append((t, xyz, quat))
                    self.get_logger().info(f"Waypoint {i + 1}/{len(plan)} [{side}]: xyz={xyz}")
                elif last_target[side] is not None:
                    # This waypoint doesn't move this side (e.g. a gripper-only step) - add
                    # a knot repeating its last real target at this time anyway, so the
                    # spline holds still (PCHIP gives exactly zero velocity between two
                    # equal-value knots - see _execute_spline) through this time span
                    # instead of interpolating straight through to whatever the NEXT real
                    # waypoint for this side turns out to be.
                    xyz, quat = last_target[side]
                    knots[side].append((t, xyz, quat))

        self._execute_spline(knots, gripper_events, start_poses, total_duration=t)
        return object_xyz

    def _execute_spline(self, knots, gripper_events, start_poses, total_duration, dt=0.02):
        """Publishes one smooth, continuous trajectory through every knot, instead of a
        straight-line segment per waypoint. Position uses PchipInterpolator (not a plain
        cubic spline): PCHIP is shape-preserving - it produces exactly zero velocity
        between two knots that hold the same value, which is what a gripper-only "hold"
        knot (see run_plan()) needs to actually mean "stay put", not just "revisit this
        point in passing". A plain natural/clamped cubic spline can overshoot slightly
        around a repeated value instead of sitting flat on it - fine for a random curve,
        not for holding still while the gripper is actively closing around the object.
        Orientation uses RotationSpline (scipy's C2-continuous rotation spline) - there's
        no PCHIP equivalent for rotations, but a bit of overshoot there is far less
        consequential than positional drift during a grasp.

        start_poses[side] = (xyz, quat) to start that side's spline from at t=0 - resolved
        by run_plan() (from _last_waypoint_pose or a live TF lookup), not re-derived here.
        """
        splines = {}
        for side, side_knots in knots.items():
            if not side_knots:
                continue
            start_xyz, start_quat = start_poses[side]

            times = np.asarray([0.0] + [knot[0] for knot in side_knots], dtype=np.float64)
            xyzs = np.asarray([start_xyz] + [knot[1] for knot in side_knots], dtype=np.float64)
            quats = np.asarray([start_quat] + [knot[2] for knot in side_knots], dtype=np.float64)

            pos_spline = PchipInterpolator(times, xyzs, axis=0)
            rot_spline = RotationSpline(times, R.from_quat(quats))
            splines[side] = (pos_spline, rot_spline)

        gripper_events = sorted(gripper_events, key=lambda e: e[0])
        next_event = 0

        start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok():
            t = min(total_duration, self.get_clock().now().nanoseconds / 1e9 - start_time)

            for side, (pos_spline, rot_spline) in splines.items():
                self.set_target(side, pos_spline(t), rot_spline(t).as_quat())

            while next_event < len(gripper_events) and gripper_events[next_event][0] <= t:
                _, side, status = gripper_events[next_event]
                self.set_gripper(side, status)
                next_event += 1

            rclpy.spin_once(self, timeout_sec=dt)
            if t >= total_duration:
                break

        for side, (pos_spline, rot_spline) in splines.items():
            self._last_waypoint_pose[side] = (
                pos_spline(total_duration).tolist(), rot_spline(total_duration).as_quat().tolist())


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
