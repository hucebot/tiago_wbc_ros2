import numpy as np
from dataclasses import dataclass
from visualization_msgs.msg import Marker


@dataclass
class ObstacleData:
    marker: Marker
    status: str  # "PENDING_ADD", "ACTIVE", "PENDING_DELETE"


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

home_config = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,  # floating_base
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_front_left_joint'
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_front_right_joint'
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_rear_left_joint'
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_rear_right_joint'
    0.21,  # 'torso_lift_joint'
    0.08,
    1.04,
    1.01,
    2.35,
    1.11,
    0.12,
    1.11,  # 'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'
    0.08,
    1.04,
    1.01,
    2.35,
    1.11,
    0.12,
    1.11,  # 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
    0.0,
    0.0,
]  # 'head_1_joint', 'head_2_joint'

tiago_pro_home_config = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,  # floating_base
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_front_left_joint'
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_front_right_joint'
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_rear_left_joint'
    np.cos(0.0),
    np.sin(0.0),  # 'wheel_rear_right_joint'
    0.35,  # 'torso_lift_joint'
    0.36,
    -1.83,
    0.47,
    -2.35,
    0.0,
    0.0,
    0.0,  # 'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint'
    -0.36,
    -1.83,
    -0.47,
    -2.35,
    0.0,
    0.0,
    0.0,  # 'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint'
    0.0,
    0.0,
]  # 'head_1_joint', 'head_2_joint'


MULTIPLE_HOME_CONFIGS_DUAL = {
    "default": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.21],
        "arm_left": [0.08, 1.04, 1.01, 2.35, 1.11, 0.12, 1.11],
        "arm_right": [0.08, 1.04, 1.01, 2.35, 1.11, 0.12, 1.11],
        "head": [0.0, 0.0],
    }
}

MULTIPLE_HOME_CONFIGS_PRO = {
    "home": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.32],
        "arm_left": [0.77, -1.83, 0.47, -2.35, 0.0, -0.08, -0.42],
        "arm_right": [-0.77, -1.83, -0.47, -2.35, 0.0, -0.08, 0.42],
        "head": [0.0, -0.67],
    },
    "table": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.34],
        "arm_left": [0.77, -1.81, 0.87, -2.18, -3.01, 1.98, 0.4618],
        "arm_right": [-2.64, -1.84, 0.47, -1.95, 2.9, 1.28, -0.037],
        "head": [0.0, -0.71],
    },
    "fridge_close": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.29],
        "arm_left": [0.77, -2.05, 0.47, -2.13, 0.0, -0.6, -0.21],
        "arm_right": [-0.77, -2.05, -0.47, -2.13, 0.0, -0.6, 0.21],
        "head": [0.0, 0.1],
    },
    "fridge_open": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.32],
        "arm_left": [0.77, -1.83, 0.47, -2.35, 0.0, -0.08, -0.42],
        "arm_right": [-0.77, -1.83, -0.47, -2.35, 0.0, -0.08, 0.42],
        "head": [0.0, 0.18],
    },
    "fridge_door": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.32],
        "arm_left": [0.77, -1.81, 0.87, -2.18, -3.01, 1.98, 0.4618],
        "arm_right": [-0.77, -2.05, -0.47, -2.13, 0.0, -0.6, 0.21],
        "head": [0.0, 0.1],
    },
    "fridge_place": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.29],
        "arm_left": [0.77, -1.81, 0.87, -2.18, -3.01, 1.98, 0.4618],
        "arm_right": [-0.77, -2.05, -0.47, -2.13, 0.0, -0.6, 0.21],
        "head": [0.0, 0.061997967546303934],
    },
    "sink": {
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
            np.cos(0.0),
            np.sin(0.0),
        ],
        "torso": [0.29],
        "arm_left": [0.77, -1.81, 0.87, -2.18, -3.01, 1.98, 0.4618],
        "arm_right": [-0.77, -2.05, -0.47, -2.13, 0.0, -0.6, 0.21],
        "head": [0.0, -0.48],
    },
    "safe_place":{
        "floating_base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        "wheels": [
                    np.cos(0.0),
                    np.sin(0.0),
                    np.cos(0.0),
                    np.sin(0.0),
                    np.cos(0.0),
                    np.sin(0.0),
                    np.cos(0.0),
                    np.sin(0.0),
                ],
        "torso": [0.17202],
        "arm_left": [1.8557, -1.5919, 0.35538, -2.0502, 0.10524, -1.5976, 0.0],
        "arm_right": [-1.8614, -1.6008, -0.34892, -1.9818, 0.10153, -1.5829, 0.0,],
        "head": [0.0, -0.32],


    }
}

DT = 1.0 / 100.0
