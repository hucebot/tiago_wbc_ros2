#!/usr/bin/env python3
import numpy as np
from typing import Dict, Optional, Any
from scipy.spatial.transform import Rotation as R

# ROS 2 Imports
import rclpy
import tf2_geometry_msgs
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Vector3, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Package info for URDF
from tiago_control_node.utils import (
    MULTIPLE_HOME_CONFIGS_DUAL,
    MULTIPLE_HOME_CONFIGS_PRO,
)
from ament_index_python.packages import get_package_share_directory

# Interactive markers (Control through RViz)
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarkerControl,
    InteractiveMarker,
    Marker,
    InteractiveMarkerFeedback,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from action_msgs.msg import GoalStatus

# For Gripper Control
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from std_srvs.srv import Trigger, SetBool

# URDF Parsing
try:
    from urdf_parser_py.urdf import URDF, Mesh, Box, Cylinder, Sphere

    HAS_URDF_PARSER = True
except ImportError:
    print("WARNING: urdf_parser_py not found. Meshes will not be loaded.")
    HAS_URDF_PARSER = False


class CartesianInterface(Node):
    def __init__(self) -> None:
        super().__init__("cartesian_interface_node")

        # --- State Variables ---
        self.teleop_mode = "rviz"
        self.base_teleop_mode = "joystick"
        self.vive_poses = {"right": None, "left": None}
        self.replay_poses = {"right": None, "left": None}
        self.marker_poses = {}
        self.task_enabled = {"right": True, "left": True}
        self.is_pressed = {"right": False, "left": False}

        # Velocity Tracking
        self.joy_twist = Twist()
        self.nav_twist = Twist()
        self.vive_twist = Twist()
        self.smoothed_twist = Twist()
        self.twist_alpha = 0.05

        # --- Parameters & Config ---
        self.declare_parameter("robot_model", "dual")
        self.declare_parameter("robot_description", "")

        self.model_type = self.get_parameter("robot_model").value
        self.urdf = self.get_parameter("robot_description").value

        if not self.urdf:
            self.get_logger().error(
                "URDF not provided via parameters. Meshes will fail."
            )
        else:
            self.get_logger().info(f"URDF received successfully for {self.model_type}!")

        # Dynamic Hardware Selection
        if self.model_type == "pro":
            self.home_configs = MULTIPLE_HOME_CONFIGS_PRO
            self.frames = {
                "right": "gripper_right_grasping_link",
                "left": "gripper_left_grasping_link",
                "base_right": "base_link",
                "base_left": "base_link",
            }
            self.gripper_open_pos = 0.0
            self.gripper_closed_pos = 0.8
        else:
            self.home_configs = MULTIPLE_HOME_CONFIGS_DUAL
            self.frames = {
                "right": "gripper_right_grasping_frame",
                "left": "gripper_left_grasping_frame",
                "base_right": "base_link",
                "base_left": "base_link",
            }
            self.gripper_open_pos = 0.0
            self.gripper_closed_pos = 0.08

        # --- URDF Parsing for Marker Meshes ---
        self.robot_urdf = None
        self.parent_map = {}

        if HAS_URDF_PARSER and self.urdf:
            self.robot_urdf = URDF.from_xml_string(self.urdf)
            for joint in self.robot_urdf.joints:
                self.parent_map[joint.child] = joint.parent

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- SUBSCRIBERS ---
        self.create_subscription(String, "/streamdeck/teleop_mode", self._mode_cb, 10)
        self.create_subscription(
            String, "/streamdeck/base_teleop_mode", self._base_mode_cb, 10
        )
        self.create_subscription(Bool, "/streamdeck/reset_config", self._reset_cb, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_subscription(Twist, "/nav/base_velocity_command", self._nav_cb, 10)
        self.create_subscription(
            Bool, "/opensot/reset_complete", self._reset_complete_cb, 1
        )

        for side in ["right", "left"]:
            self.create_subscription(
                PoseStamped,
                f"/vive/{side}/output_pose",
                lambda m, s=side: self._pose_cb("vive", s, m),
                1,
            )
            self.create_subscription(
                PoseStamped,
                f"/motion_recorder/pose_{side}",
                lambda m, s=side: self._pose_cb("replay", s, m),
                1,
            )
            self.create_subscription(
                PointStamped,
                f"/vive/{side}/gripper",
                lambda m, s=side: self._gripper_cb(m, s),
                10,
            )
            self.create_subscription(
                PointStamped,
                f"/replay/{side}/gripper",
                lambda m, s=side: self._gripper_cb(m, s),
                10,
            )

        # Vive specific subs
        self.create_subscription(
            PointStamped,
            "/vive/right/trackpad_x",
            lambda m: self._vive_trackpad_cb("vive", "right", "y", m),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/vive/right/trackpad_y",
            lambda m: self._vive_trackpad_cb("vive", "right", "x", m),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/vive/left/trackpad_x",
            lambda m: self._vive_trackpad_cb("vive", "left", "x", m),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/vive/right/trackpad_pressed",
            lambda m: self._vive_trackpad_pressed_cb("right", m),
            10,
        )
        self.create_subscription(
            PointStamped,
            "/vive/left/trackpad_pressed",
            lambda m: self._vive_trackpad_pressed_cb("left", m),
            10,
        )

        # --- ACTION CLIENTS & SERVICES ---
        self.cli_gripper_left = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_left_controller/follow_joint_trajectory",
        )
        self.cli_gripper_right = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_right_controller/follow_joint_trajectory",
        )
        self.cli_arm_left = ActionClient(
            self, FollowJointTrajectory, "/arm_left_controller/follow_joint_trajectory"
        )
        self.cli_arm_right = ActionClient(
            self, FollowJointTrajectory, "/arm_right_controller/follow_joint_trajectory"
        )
        self.cli_torso = ActionClient(
            self, FollowJointTrajectory, "/torso_controller/follow_joint_trajectory"
        )
        self.cli_head = ActionClient(
            self, FollowJointTrajectory, "/head_controller/follow_joint_trajectory"
        )

        self.cli_send_commands = self.create_client(
            SetBool, "/ros_control_bridge/send_commands"
        )

        # N Services dynamically for each home position
        self.srv_homes = {}
        for home_name, config in self.home_configs.items():
            self.srv_homes[home_name] = self.create_service(
                Trigger,
                f"home_position/{home_name}",
                # Use default arg in lambda to capture current loop iteration's config
                lambda req, res, cfg=config: self._home_service_cb(req, res, cfg),
            )

        self.gripper_state = {"left": "OPEN", "right": "OPEN"}
        self.gripper_btn_prev = {"left": 1.0, "right": 1.0}
        self.pose_synced = {"right": True, "right": False}

        # Initialize hardware to open state safely
        for side in ["right", "left"]:
            self._send_gripper(side, self.gripper_open_pos)

        # --- PUBLISHERS ---
        self.pub_home_done = self.create_publisher(
            Bool, "/cartesian_interface/home_done", 10
        )

        self.pub_target_r = self.create_publisher(
            PoseStamped, "/cartesian_interface/right/target_pose", 10
        )
        self.pub_target_l = self.create_publisher(
            PoseStamped, "/cartesian_interface/left/target_pose", 10
        )
        self.pub_target_b = self.create_publisher(
            Twist, "/cartesian_interface/base/target_twist", 10
        )
        self.pub_reset_config = self.create_publisher(
            Bool, "/streamdeck/reset_config", 10
        )
        self.pub_pause_opensot = self.create_publisher(Bool, "/opensot/pause", 10)

        # --- RVIZ MARKER SERVER ---
        self.server = InteractiveMarkerServer(self, "six_dof_marker_server")
        self.menu_handler = MenuHandler()
        self.enable_entry = self.menu_handler.insert(
            "Enable Task", callback=self._menu_cb
        )
        self.menu_handler.setCheckState(self.enable_entry, MenuHandler.CHECKED)
        self.menu_handler.insert("Reset", callback=self._menu_cb)

        self._wait_for_tf()
        self._init_marker("right")
        self._init_marker("left")

        # Control Loop (100Hz)
        self.create_timer(0.01, self._output_loop)
        self.get_logger().info("Cartesian Interface Node Initialized")

    def _osot(self, frame_id: str) -> str:
        """Helper to ensure frames cleanly map to the OpenSoT ghost tree."""
        clean_frame = frame_id.replace("opensot/", "").lstrip("/")
        return f"opensot/{clean_frame}"

    # --- HOME SERVICE LOGIC ---
    def _send_action_goal(
        self,
        part_name: str,
        client: ActionClient,
        joint_names: list,
        positions: list,
        duration_sec: float,
        retry_count: int = 0
    ) -> None:
        """Sends a Trajectory goal to a specific hardware controller with retry logic."""
        if not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"Action server for {part_name} not available!")
            self._handle_home_failure(part_name, client, joint_names, positions, duration_sec, retry_count)
            return

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start = Duration(sec=int(duration_sec), nanosec=0)

        traj.points = [p]
        goal.trajectory = traj

        self.get_logger().info(f"Sending home goal to {part_name} (Attempt {retry_count + 1}/4)...")

        future = client.send_goal_async(goal)

        # Safely bind variables to lambda to avoid late-binding loop issues
        future.add_done_callback(
            lambda f, p_name=part_name, c=client, j=joint_names, pos=positions, d=duration_sec, r=retry_count:
            self._home_goal_response_cb(f, p_name, c, j, pos, d, r)
        )

    def _home_goal_response_cb(self, future, part_name, client, joint_names, positions, duration_sec, retry_count) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Home Goal Rejected by hardware controller for {part_name}!")
            self._handle_home_failure(part_name, client, joint_names, positions, duration_sec, retry_count)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, p_name=part_name, c=client, j=joint_names, pos=positions, d=duration_sec, r=retry_count:
            self._home_result_cb(f, p_name, c, j, pos, d, r)
        )

    def _home_result_cb(self, future, part_name, client, joint_names, positions, duration_sec, retry_count) -> None:
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Hardware controller {part_name} reached home position.")
            self._check_home_completion(decrement=True)
        else:
            self.get_logger().error(f"Home Goal Failed/Aborted for {part_name} (Status code: {status})")
            self._handle_home_failure(part_name, client, joint_names, positions, duration_sec, retry_count)

    def _handle_home_failure(self, part_name, client, joint_names, positions, duration_sec, retry_count) -> None:
        """Decides whether to retry a failed action or give up."""
        MAX_RETRIES = 3

        if retry_count < MAX_RETRIES:
            self.get_logger().warn(f"Retrying {part_name} in 1 second...")

            # Create a one-shot timer that cleans itself up after triggering the retry
            timer = None
            def _timer_callback():
                nonlocal timer
                self._send_action_goal(part_name, client, joint_names, positions, duration_sec, retry_count + 1)
                timer.cancel()

            timer = self.create_timer(1.0, _timer_callback)
        else:
            self.get_logger().error(f"MAX RETRIES REACHED for {part_name}. Homing Sequence FAILED.")
            self.pub_home_done.publish(Bool(data=False))
            self._check_home_completion(decrement=True, failed=True)

    def _check_home_completion(self, decrement: bool = False, failed: bool = False) -> None:
        if failed:
            self.home_sequence_failed = True

        if decrement:
            self.home_pending_count -= 1

        self.get_logger().info(f"Home pending count: {self.home_pending_count}")

        if self.home_pending_count <= 0:
            self.home_pending_count = 0

            if self.home_sequence_failed:
                self.get_logger().error("Homing aborted due to hardware failures. Skipping OpenSoT reset.")
                # The False signal was already published to the orchestrator inside _handle_home_failure
            else:
                self.get_logger().info("All Home Actions Finished. Re-syncing OpenSoT...")
                self.pub_reset_config.publish(Bool(data=True))

    def _reset_complete_cb(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("OpenSoT Reset Confirmed. Unmuting OpenSoT...")
            self.pub_pause_opensot.publish(Bool(data=False))
            self.marker_reset_timer = self.create_timer(
                0.5, self._execute_delayed_marker_reset
            )
            self.pub_home_done.publish(Bool(data=True))

    def _call_send_commands_service(self, state: bool) -> None:
        """Pauses/Resumes the bridge to prevent OpenSoT from fighting the hardware."""
        if not self.cli_send_commands.service_is_ready():
            self.get_logger().warn(
                "ros_control_bridge/send_commands service is not ready"
            )
            return

        req = SetBool.Request()
        req.data = state
        future = self.cli_send_commands.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Bridge send_commands set to {state}: {f.result().success}"
            )
        )

    def _home_service_cb(self, request, response, target_config) -> Trigger.Response:
        self.get_logger().info("Homing Sequence Initiated...")

        self.task_enabled = {"right": False, "left": False}
        self.menu_handler.setCheckState(self.enable_entry, MenuHandler.UNCHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        for side in ["right", "left"]:
            self.vive_poses[side] = None
            self.replay_poses[side] = None
            self.pose_synced[side] = False

        self.pub_pause_opensot.publish(Bool(data=True))
        self._call_send_commands_service(False)
        self.home_pending_count = 0
        self.home_sequence_failed = False  # Track if any sub-task completely failed
        duration = 4.0

        if "arm_left" in target_config:
            jnts = [f"arm_left_{i}_joint" for i in range(1, len(target_config["arm_left"]) + 1)]
            self._send_action_goal("arm_left", self.cli_arm_left, jnts, target_config["arm_left"], duration)
            self.home_pending_count += 1

        if "arm_right" in target_config:
            jnts = [f"arm_right_{i}_joint" for i in range(1, len(target_config["arm_right"]) + 1)]
            self._send_action_goal("arm_right", self.cli_arm_right, jnts, target_config["arm_right"], duration)
            self.home_pending_count += 1

        if "torso" in target_config:
            self._send_action_goal("torso", self.cli_torso, ["torso_lift_joint"], target_config["torso"], duration)
            self.home_pending_count += 1

        if "head" in target_config:
            self._send_action_goal("head", self.cli_head, ["head_1_joint", "head_2_joint"], target_config["head"], duration)
            self.home_pending_count += 1

        response.success = True
        response.message = f"Homing started for {self.home_pending_count} controllers."
        return response

    # --- CALLBACKS ---
    def _mode_cb(self, msg: String) -> None:
        self._reset_cb(Bool(data=True))
        self.teleop_mode = msg.data
        self.get_logger().info(f"Teleop Mode -> {self.teleop_mode}")

        if self.teleop_mode == "rviz":
            # Re-spawn the markers at the robot's current physical location
            self._init_marker("right")
            self._init_marker("left")
        else:
            # Wipe the markers from RViz to prevent visual clutter and confusion
            self.server.clear()
            self.server.applyChanges()

    def _base_mode_cb(self, msg: String) -> None:
        self._reset_cb(Bool(data=True))
        self.base_teleop_mode = msg.data
        self.get_logger().info(f"Base Mode -> {self.base_teleop_mode}")

    def _reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self.task_enabled = {"right": False, "left": False}
            self.menu_handler.setCheckState(self.enable_entry, MenuHandler.UNCHECKED)
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

    def _execute_delayed_marker_reset(self) -> None:
        self.marker_reset_timer.cancel()
        for side in ["right", "left"]:
            self.vive_poses[side] = None
            self.replay_poses[side] = None
            self._reset_marker(side)

    def _pose_cb(self, source: str, side: str, msg: PoseStamped) -> None:
        if source == "vive":
            self.vive_poses[side] = msg
        elif source == "replay":
            self.replay_poses[side] = msg

    def _joy_cb(self, msg: Joy) -> None:
        if len(msg.axes) > 3:
            self.joy_twist.linear.x = msg.axes[1]
            self.joy_twist.linear.y = msg.axes[0]
            self.joy_twist.angular.z = msg.axes[2]

    def _nav_cb(self, msg: Twist) -> None:
        self.nav_twist = msg

    def _vive_trackpad_cb(
        self, source: str, side: str, axis: str, msg: PointStamped
    ) -> None:
        if source == "vive" and self.is_pressed[side]:
            if side == "right":
                if axis == "x":
                    self.vive_twist.linear.x = msg.point.x
                elif axis == "y":
                    self.vive_twist.linear.y = -1.0 * msg.point.x
            elif side == "left":
                if axis == "x":
                    self.vive_twist.angular.z = -1.0 * msg.point.x

    def _vive_trackpad_pressed_cb(self, side: str, msg: PointStamped) -> None:
        self.is_pressed[side] = msg.point.x > 0.5
        if not self.is_pressed[side]:
            if side == "right":
                self.vive_twist.linear.x = 0.0
                self.vive_twist.linear.y = 0.0
            elif side == "left":
                self.vive_twist.angular.z = 0.0

    def _gripper_cb(self, msg: PointStamped, side: str) -> None:
        desired_state = "CLOSED" if msg.point.x > 0.5 else "OPEN"

        if self.gripper_state[side] != desired_state:
            self.gripper_state[side] = desired_state
            target_pos = (
                self.gripper_closed_pos
                if desired_state == "CLOSED"
                else self.gripper_open_pos
            )

            self.get_logger().info(
                f"Sending to gripper {side}: {desired_state} ({target_pos})"
            )
            self._send_gripper(side, target_pos)

        self.gripper_btn_prev[side] = msg.point.x

    def _send_gripper(self, side: str, pos: float) -> None:
        client = self.cli_gripper_left if side == "left" else self.cli_gripper_right
        if not client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn(
                f"Gripper action server for {side} is not available! Ignoring command."
            )
            return

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        if self.model_type == "dual":
            traj.joint_names = [
                f"gripper_{side}_left_finger_joint",
                f"gripper_{side}_right_finger_joint",
            ]
            p = JointTrajectoryPoint()
            p.positions = [pos, pos]
            p.time_from_start = Duration(sec=0, nanosec=int(2e8))
            traj.points = [p]
        else:
            traj.joint_names = [f"gripper_{side}_finger_joint"]
            p = JointTrajectoryPoint()
            p.positions = [pos]
            p.time_from_start = Duration(sec=0, nanosec=int(2e8))
            traj.points = [p]

        goal.trajectory = traj
        client.send_goal_async(goal)

    # --- RVIZ MARKER & TF LOGIC ---
    def _wait_for_tf(self) -> None:
        self.get_logger().info(
            "Waiting for TF tree to populate before spawning markers..."
        )
        target_frame = self._osot(self.frames["right"])
        base_frame = self._osot(self.frames["base_right"])

        while rclpy.ok():
            if self.tf_buffer.can_transform(
                base_frame, target_frame, rclpy.time.Time()
            ):
                self.get_logger().info("TF Tree is ready!")
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def _get_fk_pose(self, side: str) -> Pose:
        base_frame = self._osot(self.frames[f"base_{side}"])
        target_frame = self._osot(self.frames[side])
        try:
            t = self.tf_buffer.lookup_transform(
                base_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except TransformException as e:
            self.get_logger().error(f"Failed to get TF for marker {side}: {e}")
            p = Pose()
            p.orientation.w = 1.0
            return p

    def _get_visual_marker(self, task_link_name: str) -> Marker:
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0.0,
            1.0,
            0.0,
            1.0,
        )

        if not HAS_URDF_PARSER or self.robot_urdf is None:
            return marker

        current_link = task_link_name
        visual_element = None
        steps = 0
        while steps < 10:
            link_obj = self.robot_urdf.link_map.get(current_link)
            if link_obj and link_obj.visual:
                visual_element = link_obj.visual
                break
            if current_link in self.parent_map:
                current_link = self.parent_map[current_link]
            else:
                break
            steps += 1

        if not visual_element:
            return marker

        geom = visual_element.geometry
        if isinstance(geom, Mesh):
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = geom.filename
            marker.scale.x = geom.scale[0] if geom.scale else 1.0
            marker.scale.y = geom.scale[1] if geom.scale else 1.0
            marker.scale.z = geom.scale[2] if geom.scale else 1.0
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                0.5,
                0.5,
                0.5,
                1.0,
            )
            marker.mesh_use_embedded_materials = True
        elif isinstance(geom, Box):
            marker.type = Marker.CUBE
            marker.scale.x, marker.scale.y, marker.scale.z = geom.size
        elif isinstance(geom, Cylinder):
            marker.type = Marker.CYLINDER
            marker.scale.x = marker.scale.y = 2.0 * geom.radius
            marker.scale.z = geom.length
        elif isinstance(geom, Sphere):
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = marker.scale.z = 2.0 * geom.radius

        try:
            t_offset = self.tf_buffer.lookup_transform(
                self._osot(task_link_name),
                self._osot(current_link),
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            T_task_to_link = np.eye(4)
            rot = [
                t_offset.transform.rotation.x,
                t_offset.transform.rotation.y,
                t_offset.transform.rotation.z,
                t_offset.transform.rotation.w,
            ]
            T_task_to_link[0:3, 0:3] = R.from_quat(rot).as_matrix()
            T_task_to_link[0:3, 3] = [
                t_offset.transform.translation.x,
                t_offset.transform.translation.y,
                t_offset.transform.translation.z,
            ]
        except TransformException:
            return marker

        T_visual_offset = np.eye(4)
        if visual_element.origin:
            pos = visual_element.origin.xyz
            rpy = visual_element.origin.rpy
            rot = R.from_euler("zyx", [rpy[2], rpy[1], rpy[0]]).as_matrix()
            T_visual_offset[0:3, 0:3] = rot
            T_visual_offset[0:3, 3] = pos

        T_final = T_task_to_link @ T_visual_offset
        marker.pose.position.x = T_final[0, 3]
        marker.pose.position.y = T_final[1, 3]
        marker.pose.position.z = T_final[2, 3]
        quat = R.from_matrix(T_final[0:3, 0:3]).as_quat()
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        return marker

    def _init_marker(self, side: str) -> None:
        m = InteractiveMarker()
        m.header.frame_id = self._osot(self.frames[f"base_{side}"])
        m.name = side
        m.scale = 0.3
        m.pose = self._get_fk_pose(side)
        self.marker_poses[side] = m.pose

        c = InteractiveMarkerControl(
            always_visible=True, interaction_mode=InteractiveMarkerControl.MENU
        )
        c.name = "menu_control"

        mesh_marker = self._get_visual_marker(self.frames[side])
        c.markers.append(mesh_marker)
        m.controls.append(c)

        for ax in ["x", "y", "z"]:
            for mode in [
                InteractiveMarkerControl.MOVE_AXIS,
                InteractiveMarkerControl.ROTATE_AXIS,
            ]:
                mc = InteractiveMarkerControl()
                mc.orientation.w = 1.0
                setattr(mc.orientation, ax, 1.0)
                mc.interaction_mode = mode
                mc.name = (
                    f"rotate_{ax}"
                    if mode == InteractiveMarkerControl.ROTATE_AXIS
                    else f"move_{ax}"
                )
                m.controls.append(mc)

        self.server.insert(marker=m, feedback_callback=self._marker_fb)
        self.menu_handler.apply(self.server, m.name)
        self.server.applyChanges()

    def _marker_fb(self, fb: InteractiveMarkerFeedback) -> None:
        self.marker_poses[fb.marker_name] = fb.pose

    def _reset_marker(self, side: str) -> None:
        new_pose = self._get_fk_pose(side)
        self.marker_poses[side] = new_pose
        self.server.setPose(side, new_pose)
        self.server.applyChanges()
        self.get_logger().info(f"Reset {side} marker to current TF pose.")

    def _menu_cb(self, fb: InteractiveMarkerFeedback) -> None:
        name = fb.marker_name
        if fb.menu_entry_id == self.enable_entry:
            st = self.menu_handler.getCheckState(self.enable_entry)
            if st == MenuHandler.CHECKED:
                self.menu_handler.setCheckState(
                    self.enable_entry, MenuHandler.UNCHECKED
                )
                self.task_enabled[name] = False
            else:
                self._reset_marker(name)
                self.menu_handler.setCheckState(self.enable_entry, MenuHandler.CHECKED)
                self.task_enabled[name] = True
        else:
            self._reset_marker(name)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def _tf_replay(self, msg: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        if msg is None:
            return None

        target_tf_frame = self._osot(target_frame)
        if self._osot(msg.header.frame_id) == target_tf_frame:
            return msg

        try:
            pose_to_transform = PoseStamped()
            pose_to_transform.header.frame_id = msg.header.frame_id
            pose_to_transform.header.stamp = rclpy.time.Time().to_msg()
            pose_to_transform.pose = msg.pose
            return self.tf_buffer.transform(pose_to_transform, target_tf_frame)
        except TransformException:
            return None

    # --- SMOOTHING & CONTROL ---
    def _scale_twist(self, twist: Twist, scale: float = 0.3) -> Twist:
        scaled = Twist()
        scaled.linear.x = twist.linear.x * scale
        scaled.linear.y = twist.linear.y * scale
        scaled.angular.z = twist.angular.z * scale
        return scaled

    def _apply_smoothing(self, target: Twist, current: Twist, alpha: float) -> Twist:
        smoothed = Twist()

        def smooth_val(curr_v, tar_v):
            val = curr_v + alpha * (tar_v - curr_v)
            return 0.0 if abs(val) < 0.001 and abs(tar_v) < 0.001 else val

        smoothed.linear.x = smooth_val(current.linear.x, target.linear.x)
        smoothed.linear.y = smooth_val(current.linear.y, target.linear.y)
        smoothed.angular.z = smooth_val(current.angular.z, target.angular.z)
        return smoothed

    def _process_arm_commands(self, side: str, pub: Any) -> None:
        if self.teleop_mode == "rviz" and not self.task_enabled[side]:
            return

        target_msg = None

        if self.teleop_mode == "rviz":
            # RViz markers are manually synced by your existing _reset_marker logic
            target_pose = self.marker_poses[side]
            self.pose_synced[side] = True
        elif self.teleop_mode == "vive":
            target_msg = self.vive_poses[side]
        elif self.teleop_mode == "replay":
            target_msg = self._tf_replay(
                self.replay_poses[side], self.frames[f"base_{side}"]
            )

        # Time-Sensitivity Check
        if target_msg is not None:
            now = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(target_msg.header.stamp)
            age = (now - msg_time).nanoseconds / 1e9  # Convert to seconds

            if age > 0.5:  # Reject messages older than 500ms
                return

            target_pose = target_msg.pose
        elif self.teleop_mode != "rviz":
            return

        if target_pose is not None:
            current_pose = self._get_fk_pose(side)
            dist = np.sqrt(
                (target_pose.position.x - current_pose.position.x) ** 2
                + (target_pose.position.y - current_pose.position.y) ** 2
                + (target_pose.position.z - current_pose.position.z) ** 2
            )

            # Spatial "Catch" Mechanism
            if not self.pose_synced[side]:
                # Require the controller/replay to get within 15cm to re-engage
                if dist < 0.15:
                    self.get_logger().info(
                        f"{side.capitalize()} arm synced! Resuming control."
                    )
                    self.pose_synced[side] = True
                else:
                    # Ignore commands until the source moves closer to the robot
                    return

            msg = PoseStamped()
            msg.header.frame_id = self._osot(self.frames[f"base_{side}"])
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = target_pose
            pub.publish(msg)

    def _process_base_commands(self) -> None:
        raw_target_b = Twist()
        if self.base_teleop_mode == "joystick":
            raw_target_b = self._scale_twist(self.joy_twist)
        elif self.base_teleop_mode == "navigation":
            raw_target_b = self.nav_twist
        elif self.base_teleop_mode == "vive":
            raw_target_b = self._scale_twist(self.vive_twist)

        self.smoothed_twist = self._apply_smoothing(
            raw_target_b, self.smoothed_twist, self.twist_alpha
        )
        self.pub_target_b.publish(self.smoothed_twist)

    def _output_loop(self) -> None:
        self._process_base_commands()
        for side, pub in [("right", self.pub_target_r), ("left", self.pub_target_l)]:
            self._process_arm_commands(side, pub)


def main():
    rclpy.init()
    node = CartesianInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
