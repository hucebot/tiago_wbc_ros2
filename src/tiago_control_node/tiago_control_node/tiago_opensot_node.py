#!/usr/bin/env python3
import os
import sys
import time
import copy
import array
import numpy as np
from scipy.spatial.transform import Rotation as R
from tiago_control_node.utils import collision_list, home_config, PKG_NAME, ObstacleData

# ROS 2 Interfaces
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Point
from ament_index_python.packages import get_package_share_directory

# OpenSoT
import pyopensot as pysot
from xbot2_interface import pyaffine3
from xbot2_interface import pyxbot2_collision
from xbot2_interface import pyxbot2_interface as xbi
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability
from pyopensot_collision.constraints.velocity import CollisionAvoidance

class TiagoOpenSoTNode(Node):
    def __init__(self):
        super().__init__('tiago_opensot_control')

        # --- Parameters ---
        self.declare_parameter('control_dt', 0.01)
        self.declare_parameter('max_jump_dist', 0.3)
        self.declare_parameter('robot_description', '')

        self.dt = self.get_parameter('control_dt').value
        self.max_jump_dist = self.get_parameter('max_jump_dist').value
        self.urdf = self.get_parameter('robot_description').value

        # Frame config
        self.frames = {
            "right": "gripper_right_grasping_frame",
            "left": "gripper_left_grasping_frame",
            "base": "base_link",
            "world": "world"
        }

        self.target_right = None
        self.target_left = None
        self.target_base_twist = Twist()
        self.needs_reset = False
        self.enable_external_obstacle = False
        self.active_collisions = {}

        # --- Subscribers/Publishers ---
        self.create_subscription(PoseStamped, '/cartesian_interface/right/target_pose', self._right_target_cb, 1)
        self.create_subscription(PoseStamped, '/cartesian_interface/left/target_pose', self._left_target_cb, 1)
        self.create_subscription(Twist, '/cartesian_interface/base/target_twist', self._base_target_cb, 10)
        self.create_subscription(Bool, '/streamdeck/reset_config', self._reset_cb, 10)
        self.create_subscription(MarkerArray, '/opensot/external_collisions', self._collision_scene_cb, 10)

        self.joint_state_publisher = self.create_publisher(JointState, '/opensot/joint_states', 10)
        self.base