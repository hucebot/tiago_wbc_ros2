#!/usr/bin/env python3
"""Automatic pick-and-place data-collection loop - Panda variant of
tiago_control_node/episode_manager.py. Identical to that file except for WHICH task module
is imported (pick_place_basket_panda instead of pick_place_basket) - PoseCommander itself is
reused directly from tiago_control_node, unmodified, following the same "edit the import to
point at a different task module" convention pose_commander.py's own main() documents.

See project memory "panda_wbc_ablation_goal" for why this exists as a full parallel of
TIAGo's own data-collection pipeline rather than a smaller one-off script: the point is to
collect Panda data through the exact same mechanism (same HDF5 schema conventions via
mujoco_sim_node.py-equivalent panda_sim_node.py, same episode_orchestrator_node.py reset
choreography, reused unmodified) so a policy trained on it is comparable to one trained on
TIAGo data.

Usage:
  ros2 run panda_control_node episode_manager --ros-args -p num_episodes:=50
"""
import rclpy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from tiago_control_node.pose_commander import PoseCommander
from panda_control_node.tasks.pick_place_basket_panda import PLAN, check_success


class EpisodeManager(PoseCommander):
    def __init__(self):
        super().__init__()
        self.declare_parameter('num_episodes', 10)
        self.num_episodes = self.get_parameter('num_episodes').value

        self.end_episode_cli = self.create_client(SetBool, '/mujoco_bridge/end_episode')
        while not self.end_episode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for /mujoco_bridge/end_episode service...")

        # See tiago_control_node/episode_manager.py's identical field for why this waits on
        # a topic rather than trusting the service response alone.
        self.episode_ready = False
        self.create_subscription(Bool, '/mujoco_bridge/episode_ready', self._episode_ready_cb, 10)

    def _episode_ready_cb(self, msg: Bool):
        if msg.data:
            self.episode_ready = True

    def _end_episode(self, success: bool) -> None:
        # See tiago_control_node/episode_manager.py's identical method for the full
        # explanation of this ordering (clear targets before spinning, clear object_xyz
        # twice) - unchanged here, this logic doesn't depend on the robot at all.
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

        self.object_xyz = None

    def run_episodes(self) -> None:
        successes = 0
        last_success = False

        for i in range(self.num_episodes):
            self.get_logger().info(f"=== Episode {i + 1}/{self.num_episodes}: resetting ===")
            self._end_episode(success=last_success)

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
