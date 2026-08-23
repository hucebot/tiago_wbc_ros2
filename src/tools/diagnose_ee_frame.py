#!/usr/bin/env python3
"""ONE-OFF DIAGNOSTIC - not part of the pipeline, safe to delete once you're done with it.

Checks whether the MuJoCo "ee_{side}" site (what mujoco_sim_node.py reads as ground truth
for ALL recorded eef_{side}_pose training data) actually matches gripper_{side}_grasping_link
(the URDF frame OpenSoT's Cartesian task targets, and what pose_commander's PLAN orientations
are defined relative to). If these two frames disagree on ORIENTATION, every orientation
value in the collected dataset is silently mislabeled relative to what actually gets
commanded - which would explain a policy whose predicted orientations look twisted/flipped
relative to what the real command frame expects, even though nothing in the training/
inference CODE is wrong.

How it works: loads its own read-only MuJoCo model (mirrors mujoco_sim_node.py's own
joint-to-qpos mapping - see TRACKED_JOINTS below), drives its qpos from the live
/joint_states topic (the running sim's real ground truth - never touches the actual running
sim's own MjData, this is a separate, independent, non-stepped instance used purely for
forward kinematics), and compares the resulting site pose against a live TF lookup of
gripper_{side}_grasping_link at the same instant, both relative to base_link.

Prints the position delta and the relative ROTATION (as an axis-angle) between the two
frames every 2s. Watch it while moving the arm around (e.g. drag the RViz marker, or run
pose_commander) rather than just once at a single pose:
  - If the printed rotation stays roughly CONSTANT across very different arm poses, that
    constant offset IS the bug - the site's orientation in the XML needs a fixed `quat`
    correction to actually match gripper_{side}_grasping_link.
  - If it's ~0 degrees everywhere, the frames already agree and this isn't the problem.
  - If it varies unpredictably with pose, something else is going on (this would be
    unusual - both readings come from the same real joint values, just through two
    different FK paths, so they should either agree or disagree by a fixed amount).

Usage (bringup.launch.py and mujoco_bridge.launch.py must already be running), from
/home/forest_ws inside the container:
  python3 src/tools/diagnose_ee_frame.py --side right
  python3 src/tools/diagnose_ee_frame.py --side left
"""
import argparse

import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

DEFAULT_XML_PATH = '/home/forest_ws/robots/pal_tiago_pro/xmls/scene_tiago_pro.xml'

# Must match mujoco_sim_node.py's own tracked joint set (torso + both arms) - these are the
# only joints whose position actually affects the ee_{side} site's forward kinematics.
TRACKED_JOINTS = (
    ['torso_lift_joint']
    + [f'arm_left_{i}_joint' for i in range(1, 8)]
    + [f'arm_right_{i}_joint' for i in range(1, 8)]
)


class FrameDiagnostic(Node):
    def __init__(self, xml_path: str, side: str):
        super().__init__('diagnose_ee_frame')
        self.side = side

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.qpos_adr = {}
        for name in TRACKED_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid == -1:
                self.get_logger().warn(f"Joint '{name}' not found in model.")
                continue
            self.qpos_adr[name] = self.model.jnt_qposadr[jid]

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'ee_{side}')
        if self.site_id == -1:
            self.get_logger().fatal(f"Site 'ee_{side}' not found in model - check the XML.")
            raise SystemExit(1)

        # ee_{side} is a direct child of arm_{side}_7_link with no rotation offset (see the
        # XML) - if TF disagrees with the site, this is what the site's pos/quat attributes
        # need to become (in arm_{side}_7_link's own local frame, which is exactly what the
        # XML's pos/quat attributes are expressed in) to make it match TF exactly instead.
        self.parent_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, f'arm_{side}_7_link')
        if self.parent_body_id == -1:
            self.get_logger().fatal(f"Body 'arm_{side}_7_link' not found in model.")
            raise SystemExit(1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_timer(2.0, self._compare)
        self.get_logger().info(
            f"Comparing MuJoCo site 'ee_{side}' vs TF 'gripper_{side}_grasping_link' every 2s "
            "- move the arm around while this runs, don't just read one sample.")

    def _joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg

    def _compare(self):
        if self.latest_joint_state is None:
            self.get_logger().info("Waiting for /joint_states...")
            return

        name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        for name, adr in self.qpos_adr.items():
            if name in name_to_pos:
                self.data.qpos[adr] = name_to_pos[name]
        mujoco.mj_forward(self.model, self.data)

        site_pos = self.data.site_xpos[self.site_id].copy()
        site_quat_wxyz = np.zeros(4)
        mujoco.mju_mat2Quat(site_quat_wxyz, self.data.site_xmat[self.site_id])
        site_quat_xyzw = site_quat_wxyz[[1, 2, 3, 0]]

        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                'base_link', f'gripper_{self.side}_grasping_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        tf_pos = np.array([tf_stamped.transform.translation.x,
                            tf_stamped.transform.translation.y,
                            tf_stamped.transform.translation.z])
        tf_quat_xyzw = np.array([tf_stamped.transform.rotation.x,
                                  tf_stamped.transform.rotation.y,
                                  tf_stamped.transform.rotation.z,
                                  tf_stamped.transform.rotation.w])

        pos_delta = site_pos - tf_pos
        # Rotation FROM the TF frame TO the MuJoCo site's frame - if the site were just a
        # translated copy of the same orientation (correct), this would be ~identity
        # (angle ~0) at every arm pose, not just this one.
        r_site = R.from_quat(site_quat_xyzw)
        r_tf = R.from_quat(tf_quat_xyzw)
        r_delta = r_tf.inv() * r_site
        angle_deg = np.degrees(r_delta.magnitude())
        axis = r_delta.as_rotvec()
        axis = axis / (np.linalg.norm(axis) + 1e-9)

        self.get_logger().info(
            f"[{self.side}] pos_delta={pos_delta.round(4).tolist()}m "
            f"(norm={np.linalg.norm(pos_delta):.4f}) | "
            f"rot_delta={angle_deg:.1f}deg about axis={axis.round(2).tolist()}"
        )

        # Exact fix: what the site's pos/quat attributes in the XML need to become (in
        # arm_{side}_7_link's own local frame - exactly what pos/quat means in the XML) so
        # the site lands exactly on the live TF pose instead. Computed via matrix
        # composition (T_local_new = T_parent_world^-1 * T_tf_world) rather than derived by
        # hand, to avoid a sign/order mistake in the correction itself.
        parent_pos = self.data.xpos[self.parent_body_id].copy()
        parent_rot = R.from_matrix(self.data.xmat[self.parent_body_id].reshape(3, 3))

        local_pos_new = parent_rot.inv().apply(tf_pos - parent_pos)
        local_rot_new = parent_rot.inv() * r_tf
        # MuJoCo XML quat attributes are (w, x, y, z) - scipy gives (x, y, z, w).
        qx, qy, qz, qw = local_rot_new.as_quat()

        self.get_logger().info(
            f"[{self.side}] -> paste into the ee_{self.side} site in the XML: "
            f'pos="{local_pos_new[0]:.5f} {local_pos_new[1]:.5f} {local_pos_new[2]:.5f}" '
            f'quat="{qw:.5f} {qx:.5f} {qy:.5f} {qz:.5f}"'
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--side', choices=['left', 'right'], required=True)
    parser.add_argument('--xml-path', default=DEFAULT_XML_PATH)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FrameDiagnostic(args.xml_path, args.side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
