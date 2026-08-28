#!/usr/bin/env python3
"""Minimal single-arm OpenSoT node for the Franka Panda.

This is deliberately a controlled ablation against tiago_pro_opensot_node.py, not a
from-scratch simpler IK - see project memory "panda_wbc_ablation_goal": the question this
whole Panda addition exists to answer is whether TIAGo's whole-body-control complexity
(dual-arm coupling, redundancy resolution, collision-avoidance interaction, mobile base) is
the source of policy-performance problems, or something else entirely. That comparison is
only meaningful if Panda goes through the SAME solver library and the SAME general
control-loop shape (subscribe to a Cartesian target -> build a stack -> stack.update() ->
solver.solve() -> integrate q -> publish joint targets) as tiago_pro_opensot_node.py, just
with a far simpler stack: one Cartesian task for the gripper, joint/velocity limits, a light
Postural regularizer - no floating base, no dual-arm, no Gaze, no collision-avoidance JSON.

URDF: robots/panda/urdf/panda.urdf - vendored from OpenSoT's own bundled Panda/FR3 IK
example (external/OpenSoT/tests/robots/panda/panda.urdf), with joint/link names renamed
from fp3_joint*/fp3_link* to joint*/link* to match the MuJoCo model's own joint names (see
that file's header comment), plus two extra fixed links (hand, ee_panda) added so this
node's Cartesian task target frame is the exact same pose as what
robots/panda/xmls/panda.xml's "ee_panda" MuJoCo site reports - targeting the URDF's bare
flange (link8) instead would silently solve for the wrong orientation by a constant
45-degree-ish offset (see that file's own comment for the derivation).

Fixed-base, single 7-DOF arm: unlike tiago_pro_opensot_node.py, there's no floating-base
q[0:7]/wheel-index slicing to handle - q is simply the 7 arm joint angles, in the model's
own joint order (model.getJointNames() already returns exactly those 7, no [1:] skip
needed). The gripper (2 finger joints) is not part of this stack at all, same as TIAGo:
open/close is a direct topic command handled entirely by the MuJoCo bridge, not solved for.

Reset behavior is simpler than tiago_pro_opensot_node.py's wait_for_initial_state() too:
that mechanism exists mainly for the real-robot bootstrap case (read actual encoder
positions before any orchestration exists). Here, episode_orchestrator_node.py's own
sequencing already guarantees the sim is AT home (via panda_sim_node.py's
reset_robot_home) before it publishes /streamdeck/reset_config - by the time this node
reacts to that message, snapping q straight to HOME_POSITIONS is already correct, no
external state to read back first.
"""
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import pyopensot as pysot
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
from pyopensot.tasks.velocity import Postural, Cartesian

# Same values as robots/panda/xmls/panda.xml's own <keyframe name="home"> arm qpos, and
# panda_mujoco_bridge/panda_sim_node.py's own HOME_POSITIONS - kept in sync by hand since
# MuJoCo and OpenSoT don't share a config format. A standard Franka "ready" posture (elbow
# up, gripper pointing down), not an arbitrary choice. This is also the posture
# pick_place_basket_panda.py's TOP_DOWN constant was numerically derived against - it must
# stay in sync with this value or TOP_DOWN stops being an actual top-down orientation.
HOME_POSITIONS = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]


class PandaOpenSotNode(Node):
    def __init__(self):
        super().__init__('panda_opensot_control')

        self.declare_parameter('control_dt', 0.01)
        self.declare_parameter('urdf_path', '/home/forest_ws/robots/panda/urdf/panda.urdf')
        self.declare_parameter('lambdas.gripper', 0.1)
        self.declare_parameter('lambdas.postural', 0.01)
        self.declare_parameter('frames.base', 'link0')
        self.declare_parameter('frames.gripper', 'ee_panda')
        # pose_commander.py/episode_manager.py are reused as-is from tiago_control_node
        # (see their own docstrings - genuinely robot-agnostic) and always publish through
        # the 'right' side of their {'right','left'} dicts for a single-arm robot, exactly
        # like TIAGo Pro's own pick_place_basket task already only drives 'right' - so this
        # listens on the same topic name, not a Panda-specific one.
        self.declare_parameter('target_topic', '/cartesian_interface/right/target_pose')

        self.dt = self.get_parameter('control_dt').value
        self.urdf_path = self.get_parameter('urdf_path').value
        self.l_gripper = self.get_parameter('lambdas.gripper').value
        self.l_postural = self.get_parameter('lambdas.postural').value
        self.frame_base = self.get_parameter('frames.base').value
        self.frame_gripper = self.get_parameter('frames.gripper').value
        target_topic = self.get_parameter('target_topic').value

        with open(self.urdf_path, 'r') as f:
            self.urdf = f.read()

        self.target = None
        self.needs_reset = False

        self.create_subscription(PoseStamped, target_topic, self._target_cb, 10)
        self.create_subscription(Bool, '/streamdeck/reset_config', self._reset_cb, 10)

        self.joint_state_pub = self.create_publisher(JointState, '/opensot/joint_states', 10)
        self.reset_ok_pub = self.create_publisher(Bool, '/opensot/reset_complete', 1)

        self.get_logger().info(
            f"Panda OpenSoT control node initialized (urdf={self.urdf_path}, "
            f"target_topic={target_topic}).")

    def _target_cb(self, msg: PoseStamped):
        self.target = msg

    def _reset_cb(self, msg: Bool):
        if msg.data:
            self.needs_reset = True
            self.target = None


def main(args=None):
    rclpy.init(args=args)
    node = PandaOpenSotNode()

    model = xbi.ModelInterface2(node.urdf)
    q = np.array(HOME_POSITIONS, dtype=np.float64)
    model.setJointPosition(q)
    model.update()

    qmin, qmax = model.getJointLimits()
    qlims = JointLimits(model, qmax, qmin)
    dqmax = model.getVelocityLimits()
    dqlims = VelocityLimits(model, dqmax, node.dt)

    gripper = Cartesian("gripper", model, node.frame_gripper, node.frame_base)
    gripper.setLambda(node.l_gripper)

    postural = Postural(model)
    postural.setLambda(node.l_postural)

    # Single Cartesian task (left of '/') at top priority, postural regularizer only in the
    # leftover null-space - the same '/' priority syntax tiago_pro_opensot_node.py uses,
    # just with one task instead of TIAGo's left+right+base composition.
    stack = (gripper / postural) << qlims << dqlims
    solver = pysot.iHQP(stack)

    msg = JointState()
    msg.name = model.getJointNames()

    try:
        while rclpy.ok():
            start = time.perf_counter()

            if node.needs_reset:
                node.get_logger().info("Resetting Panda OpenSoT model to home...")
                q = np.array(HOME_POSITIONS, dtype=np.float64)
                model.setJointPosition(q)
                model.update()
                gripper.reset()
                postural.reset()
                node.needs_reset = False
                node.reset_ok_pub.publish(Bool(data=True))

            model.setJointPosition(q)
            model.update()

            if node.target is not None:
                p_ref = gripper.getReference()[0]
                p_ref.translation = [
                    node.target.pose.position.x,
                    node.target.pose.position.y,
                    node.target.pose.position.z,
                ]
                p_ref.linear = R.from_quat([
                    node.target.pose.orientation.x, node.target.pose.orientation.y,
                    node.target.pose.orientation.z, node.target.pose.orientation.w,
                ]).as_matrix()
                gripper.setReference(p_ref, np.zeros(6))
            else:
                gripper.reset()

            stack.update()

            dq = np.zeros(model.getNv())
            try:
                dq = solver.solve()
            except Exception as e:
                node.get_logger().error(f"Solver failed: {e}", throttle_duration_sec=1.0)

            q = model.sum(q, dq)

            msg.position = q.tolist()
            msg.header.stamp = node.get_clock().now().to_msg()
            node.joint_state_pub.publish(msg)

            rclpy.spin_once(node, timeout_sec=0)

            elapsed = time.perf_counter() - start
            if elapsed < node.dt:
                time.sleep(node.dt - elapsed)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
