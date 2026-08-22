#!/usr/bin/env python3
"""Automatic pick-and-place data-collection loop.

Repeats, for `num_episodes`: reset the MuJoCo episode (robot back to home, the
target object respawned at a new random table position via the MuJoCo bridge's
/mujoco_bridge/end_episode service - see tiago_pro_mujoco_bridge/mujoco_bridge_node.py),
run pose_commander's PLAN against wherever the object actually is, then judge
success from whether the object ends up inside the basket fixture.

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

from tiago_control_node.pose_commander import PoseCommander, PLAN, BASKET_XY

SUCCESS_XY_TOLERANCE = 0.05   # meters: how close to the basket's center counts as "inside" (basket's
                               # inner cavity is roughly +-0.07m, see scene_tiago_pro.xml)
SUCCESS_Z_MAX = 0.60          # object must have settled into/near the basket, not still airborne


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

        # Now it's actually safe to proceed: invalidate our cached object pose so the next
        # run_plan() blocks for a genuinely fresh (post-respawn) one, and stop our own
        # publish timer from re-sending the previous episode's last waypoint, which would
        # otherwise immediately re-drag the arm away from the fresh reset.
        self.object_xyz = None
        self.clear_targets()

    def _check_success(self) -> bool:
        target_x, target_y = BASKET_XY

        fx, fy, fz = self.object_xyz
        dist = ((fx - target_x) ** 2 + (fy - target_y) ** 2) ** 0.5
        success = dist < SUCCESS_XY_TOLERANCE and fz < SUCCESS_Z_MAX
        self.get_logger().info(
            f"Object ended at ({fx:.3f}, {fy:.3f}, {fz:.3f}), basket at ({target_x:.3f}, {target_y:.3f}), "
            f"dist={dist:.3f} -> {'SUCCESS' if success else 'FAILURE'}")
        return success

    def run_episodes(self) -> None:
        successes = 0
        last_success = False

        for i in range(self.num_episodes):
            self.get_logger().info(f"=== Episode {i + 1}/{self.num_episodes}: resetting ===")
            self._end_episode(success=last_success)

            self.run_plan(PLAN)
            last_success = self._check_success()
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
