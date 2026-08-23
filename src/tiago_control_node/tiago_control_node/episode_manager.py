#!/usr/bin/env python3
"""Automatic pick-and-place data-collection loop.

Repeats, for `num_episodes`: reset the MuJoCo episode (robot back to home, the
target object respawned at a new random table position via the MuJoCo bridge's
/mujoco_bridge/end_episode service - see tiago_pro_mujoco_bridge/episode_orchestrator_node.py),
run the task's PLAN against wherever the object actually is, then judge success via
the task's check_success(). The task itself (what to do, how to tell it worked) lives in
tiago_control_node/tasks/ - this file is generic across whatever task is imported below.

Each successful episode's full step-by-step trajectory is saved to HDF5 by the MuJoCo
bridge itself as a "demo_N" group in a single file (its 'episode_log_path' parameter);
failed episodes are discarded by default (see the bridge's 'save_failed_episodes' param).
This node only drives the task and reports the outcome for the bridge to record; it does
not touch HDF5 directly.

Usage:
  ros2 run tiago_control_node episode_manager --ros-args -p num_episodes:=50
"""
import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from tiago_control_node.pose_commander import PoseCommander
from tiago_control_node.tasks.pick_place_basket import PLAN, check_success


class EpisodeManager(PoseCommander):
    def __init__(self):
        super().__init__()
        self.declare_parameter('num_episodes', 10)
        self.num_episodes = self.get_parameter('num_episodes').value

        self.end_episode_cli = self.create_client(SetBool, '/mujoco_bridge/end_episode')
        while not self.end_episode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /mujoco_bridge/end_episode service...")

        # The service response only means "reset requested" - the robot settle and object
        # respawn both finish asynchronously afterward on the bridge side, signaled by this
        # topic. Without waiting for it, run_plan() starts immediately and grabs whatever
        # object position happens to be published at that instant - almost always the stale
        # pre-reset one, while the arm is still mid-resync.
        self.episode_ready = False
        self.create_subscription(Bool, '/mujoco_bridge/episode_ready', self._episode_ready_cb, 10)

    def _episode_ready_cb(self, msg: Bool):
        if msg.data:
            self.episode_ready = True

    def _end_episode(self, success: bool) -> None:
        # MUST happen before any spinning below, not after: spin_until_future_complete /
        # spin_once service every ready callback on this node, including our own
        # _publish_targets timer - if self.targets still held the just-finished plan's
        # final waypoint, that timer would keep republishing it to
        # /cartesian_interface/{side}/target_pose for the entire multi-second reset,
        # continuously fighting tiago_pro_opensot_node's attempt to actually reach home
        # (its reset_poses() clears target_right/left once, but the very next stale
        # message right behind it immediately sets it again). That's what was causing the
        # arm to get stuck instead of settling - not a timing race, a sustained fight for
        # the whole reset window. Clearing this first means there's nothing stale left to
        # republish by the time any spinning starts.
        self.object_xyz = None
        self.clear_targets()

        self.episode_ready = False
        req = SetBool.Request()
        req.data = success
        future = self.end_episode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"end_episode(success={success}) -> {future.result().message}")

        self.get_logger().info("Waiting for /mujoco_bridge/episode_ready (robot settled, object respawned)...")
        end_time = self.get_clock().now().nanoseconds / 1e9 + 10.0
        while rclpy.ok() and not self.episode_ready and self.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.episode_ready:
            raise RuntimeError(
                "Timed out waiting for /mujoco_bridge/episode_ready - is the MuJoCo bridge running "
                "and receiving /opensot/reset_complete?")

        # Clear this AGAIN, now that episode_ready is confirmed: episode_ready and the
        # freshly-randomized /mujoco_bridge/target_object_pose come from two different
        # processes (episode_orchestrator_node and mujoco_sim_node respectively), so there's
        # no guarantee we received them in that order even though the orchestrator only
        # publishes ready after randomizing server-side - if the stale (still-parked)
        # reading from earlier in the reset happened to arrive here last, self.object_xyz
        # would already be non-None and run_plan()'s wait_for_object_pose() would return
        # immediately with that wrong position instead of waiting for anything fresh. This
        # guarantees the very next reading run_plan() sees was published after we already
        # know the reset (and therefore the randomize) is done.
        self.object_xyz = None

    def run_episodes(self) -> None:
        successes = 0
        last_success = False

        for i in range(self.num_episodes):
            self.get_logger().info(f"=== Episode {i + 1}/{self.num_episodes}: resetting ===")
            self._end_episode(success=last_success)

            # One bad episode (a plan step timing out, a transient service failure, ...)
            # shouldn't kill an otherwise-long unattended data-collection run - log it,
            # count it as a failure, and move on to the next reset.
            try:
                pick_xyz = self.run_plan(PLAN)
                last_success, message = check_success(self.object_xyz, pick_xyz)
                self.get_logger().info(message)
            except Exception as exc:
                self.get_logger().error(f"Episode {i + 1} failed: {exc!r}")
                last_success = False

            successes += int(last_success)

        self._end_episode(success=last_success)
        self.get_logger().info(f"Done: {successes}/{self.num_episodes} episodes succeeded.")


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeManager()

    try:
        node.run_episodes()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
