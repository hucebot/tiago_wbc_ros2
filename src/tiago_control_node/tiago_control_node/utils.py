import os
from dataclasses import dataclass

from visualization_msgs.msg import Marker


@dataclass
class ObstacleData:
    marker: Marker
    status: str  # "PENDING_ADD", "ACTIVE", "PENDING_DELETE"


def load_home_poses(robot_model: str) -> dict:
    """Return the named home poses for ``robot_model`` ("pro" or "dual").

    Reads ``config/home_poses.yaml`` from the installed package share directory.
    Each top value is ``{torso, arm_left, arm_right, head}``. Raises RuntimeError
    if the file or the robot's section is missing.
    """
    import yaml
    from ament_index_python.packages import get_package_share_directory

    path = os.path.join(
        get_package_share_directory("tiago_control_node"), "config", "home_poses.yaml"
    )
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except OSError as e:
        raise RuntimeError(f"Could not read home poses at {path}: {e}") from e

    key = "pro" if robot_model == "pro" else "dual"
    poses = data.get(key)
    if not poses:
        raise RuntimeError(f"home_poses.yaml has no '{key}' section ({path})")
    return poses


# Tikhonov regularisation for the iHQP solver. Large on purpose: keeps the
# hierarchy well-conditioned and joint velocities small near singularities, at
# the cost of some tracking accuracy.
EPS_REGULARISATION = 1e10


def q_index_map(model) -> dict:
    """joint name -> start index in the configuration vector q.

    q = [3 base translation][4 base quaternion] then one slot per DOF, except
    continuous 'wheel' joints which use 2 slots (cos, sin). Use this for anything
    that indexes into getJointPosition() / the q vector.
    """
    idx, out = 7, {}
    for name in model.getJointNames():
        if name == "reference":
            continue
        out[name] = idx
        idx += 2 if "wheel" in name else 1
    return out


def v_index_map(model) -> dict:
    """joint name -> index in velocity space.

    Covers dq and the nv-sized vectors from getJointLimits() / getVelocityLimits():
    6 floating-base DOF, then 1 per joint (continuous 'wheel' joints are 1 DOF here,
    unlike in q).
    """
    idx, out = 6, {}
    for name in model.getJointNames():
        if name == "reference":
            continue
        out[name] = idx
        idx += 1
    return out


collision_list = {
    ("arm_left_3_link", "base_link"),
    ("arm_left_5_link", "base_link"),
    ("gripper_left_left_finger_link", "base_link"),
    ("gripper_left_right_finger_link", "base_link"),
    ("gripper_left_link", "base_link"),
    ("arm_left_3_link", "head_2_link"),
    ("arm_left_5_link", "head_2_link"),
    ("gripper_left_left_finger_link", "head_2_link"),
    ("gripper_left_right_finger_link", "head_2_link"),
    ("gripper_left_link", "head_2_link"),
    ("arm_left_3_link", "torso_lift_link"),
    ("arm_left_5_link", "torso_lift_link"),
    ("gripper_left_left_finger_link", "torso_lift_link"),
    ("gripper_left_right_finger_link", "torso_lift_link"),
    ("gripper_left_link", "torso_lift_link"),
    ("arm_right_3_link", "base_link"),
    ("arm_right_5_link", "base_link"),
    ("gripper_right_left_finger_link", "base_link"),
    ("gripper_right_right_finger_link", "base_link"),
    ("gripper_right_link", "base_link"),
    ("arm_right_3_link", "head_2_link"),
    ("arm_right_5_link", "head_2_link"),
    ("gripper_right_left_finger_link", "head_2_link"),
    ("gripper_right_right_finger_link", "head_2_link"),
    ("gripper_right_link", "head_2_link"),
    ("arm_right_3_link", "torso_lift_link"),
    ("arm_right_5_link", "torso_lift_link"),
    ("gripper_right_left_finger_link", "torso_lift_link"),
    ("gripper_right_right_finger_link", "torso_lift_link"),
    ("gripper_right_link", "torso_lift_link"),
    ("gripper_right_left_finger_link", "gripper_left_left_finger_link"),
    ("gripper_right_left_finger_link", "gripper_left_right_finger_link"),
    ("gripper_right_right_finger_link", "gripper_left_left_finger_link"),
    ("gripper_right_right_finger_link", "gripper_left_right_finger_link"),
    ("gripper_right_left_finger_link", "gripper_left_link"),
    ("gripper_right_right_finger_link", "gripper_left_link"),
    ("gripper_left_left_finger_link", "gripper_right_link"),
    ("gripper_left_right_finger_link", "gripper_right_link"),
    ("gripper_left_link", "gripper_right_link"),
    ("gripper_left_link", "arm_right_5_link"),
    ("gripper_right_link", "arm_left_5_link"),
    ("arm_left_5_link", "arm_right_5_link"),
    ("arm_left_5_link", "arm_right_4_link"),
    ("arm_left_4_link", "arm_right_5_link"),
    ("gripper_left_link", "arm_right_4_link"),
    ("gripper_left_link", "arm_right_5_link"),
    ("gripper_right_link", "arm_left_4_link"),
    ("gripper_right_link", "arm_left_5_link"),
    ("torso_fixed_column_link", "gripper_right_left_finger_link"),
    ("torso_fixed_column_link", "gripper_right_right_finger_link"),
    ("torso_fixed_column_link", "gripper_right_link"),
    ("torso_fixed_column_link", "arm_right_6_link"),
    ("torso_fixed_column_link", "arm_right_5_link"),
    ("torso_fixed_column_link", "arm_right_4_link"),
    ("torso_fixed_column_link", "arm_right_3_link"),
    ("torso_fixed_column_link", "gripper_left_right_finger_link"),
    ("torso_fixed_column_link", "gripper_left_right_finger_link"),
    ("torso_fixed_column_link", "gripper_left_link"),
    ("torso_fixed_column_link", "arm_left_6_link"),
    ("torso_fixed_column_link", "arm_left_5_link"),
    ("torso_fixed_column_link", "arm_left_4_link"),
    ("torso_fixed_column_link", "arm_left_3_link"),
}

# Named home poses moved to config/home_poses.yaml -- use load_home_poses().
