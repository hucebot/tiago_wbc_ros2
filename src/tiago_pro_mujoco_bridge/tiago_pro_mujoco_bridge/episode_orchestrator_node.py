#!/usr/bin/env python3
"""Owns the episode reset choreography as an explicit, linear sequence.

External interface (used by episode_manager.py):
  - /mujoco_bridge/end_episode (SetBool, request.data=success of the episode that just
    finished): triggers the full reset sequence below. Returns once reset is REQUESTED, not
    once it's done - see /mujoco_bridge/episode_ready.
  - /mujoco_bridge/episode_ready (Bool, published): fires once the whole sequence below has
    actually finished - robot settled at home AND object respawned. Callers must wait for
    this before trusting /mujoco_bridge/target_object_pose or issuing new commands, or
    they'll act on the stale pre-reset object position while the arm is still mid-resync.

end_episode's sequence, each step waiting for the previous to finish before starting the next:
  1. save the just-finished episode's log (mujoco_sim_node.py's
     /mujoco_bridge/sim/save_episode_log)
  2. reset the robot to home (mujoco_sim_node.py's /mujoco_bridge/sim/reset_robot_home) -
     this is where MuJoCo's mj_resetData side effect leaves the object at its known-safe
     XML default position, deliberately NOT yet randomized (see step 5)
  3. tell tiago_pro_opensot_node to drop its cached cartesian targets and resync its solver
     state (/streamdeck/reset_config) and wait for it to confirm (/opensot/reset_complete),
     with a MAX_RESET_WAIT_SEC fallback so a crashed/absent opensot node can't hang this forever
  4. confirm the robot has ACTUALLY settled at home - wait for /mujoco_bridge/settled_at_home
     (published by mujoco_sim_node.py, checking real qpos/qvel against HOME_POSITIONS), with
     its own MAX_SETTLE_WAIT_SEC fallback. Needed because step 2 teleports qpos straight to
     home, but nothing stops a stale (pre-reset) /opensot/joint_states message from landing
     in the gap before step 3's resync actually completes and pulling a joint away again -
     /opensot/reset_complete alone only confirms OpenSoT's OWN state resynced, not that
     MuJoCo's actual joint state is still (or ever) at home.
  5. only now respawn the object at a new random table position (mujoco_sim_node.py's
     /mujoco_bridge/sim/randomize_object_pose) - randomizing it any earlier risks the newly
     spawned object overlapping the not-yet-settled arm, which MuJoCo's contact solver
     resolves by violently flinging them apart on the next physics step
  6. announce /mujoco_bridge/episode_ready - only now is a caller guaranteed the robot is
     both settled at home AND the object is in its fresh randomized position

The end_episode service runs on a ReentrantCallbackGroup, spun by a MultiThreadedExecutor in
main() - its handler blocks on 3 outgoing service calls to mujoco_sim_node.py plus a wait on
an incoming topic, so a single-threaded executor would deadlock: it can never process those
responses/messages, because it's busy running this very handler while it waits on them.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool

# Safety net if /opensot/reset_complete never arrives (e.g. tiago_pro_opensot_node isn't
# running or crashed mid-reset) - without this, a hung reset would block end_episode forever.
MAX_RESET_WAIT_SEC = 5.0

# Safety net if /mujoco_bridge/settled_at_home never goes true (e.g. mujoco_sim_node crashed,
# or something is persistently fighting the reset) - without this, a genuinely stuck robot
# would block end_episode forever instead of surfacing the problem.
MAX_SETTLE_WAIT_SEC = 5.0

# Safety net for the sim/* service calls to mujoco_sim_node - these are local, near-instant
# calls, so this only ever fires if something is actually broken (node crashed, etc.).
SIM_SERVICE_CALL_TIMEOUT_SEC = 10.0


class EpisodeOrchestratorNode(Node):
    def __init__(self):
        super().__init__('episode_orchestrator_node')

        # Every callback below (the service, the client responses, the subscription) shares
        # this one group so they're all allowed to interleave with each other - see the
        # deadlock note in the module docstring.
        cb_group = ReentrantCallbackGroup()

        self.reset_robot_home_cli = self.create_client(
            Trigger, '/mujoco_bridge/sim/reset_robot_home', callback_group=cb_group)
        self.randomize_object_pose_cli = self.create_client(
            Trigger, '/mujoco_bridge/sim/randomize_object_pose', callback_group=cb_group)
        self.save_episode_log_cli = self.create_client(
            SetBool, '/mujoco_bridge/sim/save_episode_log', callback_group=cb_group)
        for name, cli in (
            ('reset_robot_home', self.reset_robot_home_cli),
            ('randomize_object_pose', self.randomize_object_pose_cli),
            ('save_episode_log', self.save_episode_log_cli),
        ):
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"Waiting for mujoco_sim_node's /mujoco_bridge/sim/{name} service...")

        self.reset_config_pub = self.create_publisher(Bool, '/streamdeck/reset_config', 10)
        self.episode_ready_pub = self.create_publisher(Bool, '/mujoco_bridge/episode_ready', 10)

        self._reset_complete = False
        self.create_subscription(
            Bool, '/opensot/reset_complete', self._reset_complete_cb, 10, callback_group=cb_group)

        self._settled_at_home = False
        self.create_subscription(
            Bool, '/mujoco_bridge/settled_at_home', self._settled_at_home_cb, 10, callback_group=cb_group)

        self.create_service(
            SetBool, '/mujoco_bridge/end_episode', self._end_episode_cb, callback_group=cb_group)

        self.episode_idx = 0  # just for this node's own phase-log lines below
        self.get_logger().info("Episode orchestrator ready.")

    def _reset_complete_cb(self, msg: Bool):
        if msg.data:
            self._reset_complete = True

    def _settled_at_home_cb(self, msg: Bool):
        self._settled_at_home = msg.data

    def _call(self, cli, request, timeout_sec=SIM_SERVICE_CALL_TIMEOUT_SEC):
        """Blocks this callback until `future` resolves, WITHOUT calling
        rclpy.spin_until_future_complete() - that would add this node to a second,
        internally-created executor while main()'s MultiThreadedExecutor is already
        spinning it, which is unsupported and hangs instead of deadlocking cleanly (this is
        exactly what caused the very first /mujoco_bridge/end_episode call to hang). Since
        the node is already being spun by multiple executor threads, another one of them
        picks up the response callback while this thread just polls - no extra spin needed."""
        future = cli.call_async(request)
        end_time = time.time() + timeout_sec
        while rclpy.ok() and not future.done() and time.time() < end_time:
            time.sleep(0.01)
        if not future.done():
            self.get_logger().error(f"Call to {cli.srv_name} timed out after {timeout_sec}s.")
            return None
        return future.result()

    def _end_episode_cb(self, request, response):
        self.episode_idx += 1
        self.get_logger().info(f"=== Episode {self.episode_idx}: reset requested (success={request.data}) ===")

        save_result = self._call(self.save_episode_log_cli, SetBool.Request(data=request.data))
        if save_result is not None:
            self.get_logger().info(f"Saved: {save_result.message}")

        self._call(self.reset_robot_home_cli, Trigger.Request())
        self.get_logger().info("Robot reset to home, waiting for OpenSoT resync...")

        self._reset_complete = False
        self.reset_config_pub.publish(Bool(data=True))
        end_time = self.get_clock().now().nanoseconds / 1e9 + MAX_RESET_WAIT_SEC
        while rclpy.ok() and not self._reset_complete and self.get_clock().now().nanoseconds / 1e9 < end_time:
            time.sleep(0.05)
        if not self._reset_complete:
            self.get_logger().warn(
                f"No /opensot/reset_complete after {MAX_RESET_WAIT_SEC}s - is tiago_pro_opensot_node "
                "running? Proceeding anyway so episodes aren't silently stuck.")

        # OpenSoT confirming ITS OWN state resynced doesn't guarantee MuJoCo's actual joint
        # state is still (or ever) at home - a stale /opensot/joint_states message could have
        # landed and pulled a joint away again in the gap before that resync completed. Poll
        # the sim's own live qpos/qvel check instead of trusting reset_complete alone - this is
        # the actual "start recording only once we're 100% at home" guarantee.
        self._settled_at_home = False
        end_time = self.get_clock().now().nanoseconds / 1e9 + MAX_SETTLE_WAIT_SEC
        while rclpy.ok() and not self._settled_at_home and self.get_clock().now().nanoseconds / 1e9 < end_time:
            time.sleep(0.05)
        if not self._settled_at_home:
            self.get_logger().warn(
                f"Robot did not settle at home within {MAX_SETTLE_WAIT_SEC}s - is something "
                "still commanding it away from home? Proceeding anyway so episodes aren't "
                "silently stuck, but this episode's early steps may not start exactly at home.")
        else:
            self.get_logger().info("Robot confirmed settled at home.")

        randomize_result = self._call(self.randomize_object_pose_cli, Trigger.Request())
        self.get_logger().info(
            f"Object respawned: {randomize_result.message if randomize_result else 'no response'}")

        self.episode_ready_pub.publish(Bool(data=True))
        self.get_logger().info(f"=== Episode {self.episode_idx}: ready ===")

        response.success = True
        response.message = save_result.message if save_result is not None else "reset complete"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeOrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
