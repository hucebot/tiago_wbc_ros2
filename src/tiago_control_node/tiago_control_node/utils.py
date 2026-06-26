import numpy as np
from dataclasses import dataclass
from visualization_msgs.msg import Marker


@dataclass
class ObstacleData:
    marker: Marker
    status: str # "PENDING_ADD", "ACTIVE", "PENDING_DELETE"


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

home_config = [0., 0., 0., .0, 0., 0., 1., # floating_base
     np.cos(0.), np.sin(0.),     # 'wheel_front_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_front_right_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_right_joint'
     0.21,                # 'torso_lift_joint'
     0.08, 1.04, 1.01, 2.35, 1.11, 0.12, 1.11, # 'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'
     0.08, 1.04, 1.01, 2.35, 1.11, 0.12, 1.11, # 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
     0., 0.] # 'head_1_joint', 'head_2_joint'

tiago_pro_home_config = [0., 0., 0., .0, 0., 0., 1., # floating_base
     np.cos(0.), np.sin(0.),     # 'wheel_front_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_front_right_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_left_joint'
     np.cos(0.), np.sin(0.),     # 'wheel_rear_right_joint'
     0.35,                # 'torso_lift_joint'
     0.36, -1.83, 0.47, -2.35, 0.0, 0.0, 0.0, # 'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'
     -0.36, -1.83, -0.47, -2.35, 0.0, 0.0, 0.0, # 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
     0., 0.] # 'head_1_joint', 'head_2_joint'



# Dictionaries for the Action Controllers (cartesian_interface_node)
HOME_CONFIG_DUAL = {
    'torso': [0.21],
    'arm_left': [0.08, 1.04, 1.01, 2.35, 1.11, 0.12, 1.11],
    'arm_right': [0.08, 1.04, 1.01, 2.35, 1.11, 0.12, 1.11],
    'head': [0.0, 0.0]
}

# NOTE: If the real Tiago Pro has 8-DoF arms or different joints,
# update these arrays to match the physical hardware!
HOME_CONFIG_PRO = {
    'torso': [0.32],
    'arm_left': [0.77, -1.83, 0.47, -2.35, 0.0, -0.08, -0.42],
    'arm_right': [-0.77, -1.83, -0.47, -2.35, 0.0, -0.08, 0.42],
    'head': [0.0, -0.67]
}

DT = 1.0/100.0