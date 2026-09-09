#!/usr/bin/env python3
from typing import Any

import numpy as np

# ROS 2 Imports
import rclpy
import tf2_geometry_msgs  # noqa: F401  (side-effect: registers geometry_msgs <-> tf2 conversions)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Twist, Vector3
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

# Interactive markers (Control through RViz)
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from tiago_control_node.utils import load_home_poses

try:
    from urdf_parser_py.urdf import URDF, Box, Cylinder, Mesh, Sphere

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
        self._fk_cache = {}  # last good FK pose per side (fallback on a TF miss)
        self.marker_reset_timer = None  # one-shot: deferred marker snap after homing
        self._marker_reset_tries = 0
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
        self.declare_parameter("joy.scale_linear", 0.3)
        self.declare_parameter("joy.scale_angular", 0.3)
        self.model_type = self.get_parameter("robot_model").value
        self.urdf = self.get_parameter("robot_description").value
        self.joy_scale_linear = self.get_parameter("joy.scale_linear").value
        self.joy_scale_angular = self.get_parameter("joy.scale_angular").value

        if not self.urdf:
            self.get_logger().error("URDF not provided via parameters. Meshes will fail.")
        else:
            self.get_logger().info(f"URDF received successfully for {self.model_type}!")

        self.home_configs = load_home_poses("pro" if self.model_type == "pro" else "dual")

        if self.model_type == "pro":
            self.frames = {
                "right": "gripper_right_grasping_link",
                "left": "gripper_left_grasping_link",
                "base_right": "base_link",
                "base_left": "base_link",
            }
            self.gripper_open_pos = 0.0
            self.gripper_closed_pos = 0.8
        else:
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
        # "opensot/*" and "cartesian_interface/*" are RELATIVE -> they follow the
        # launch `namespace` and pair with the solver. Foreign sources
        # ("/streamdeck/*", "/joy", "/vive/*", "/motion_recorder/*", "/replay/*")
        # and the ros2_control gripper topics stay ABSOLUTE.
        self.create_subscription(String, "/streamdeck/teleop_mode", self._mode_cb, 10)
        self.create_subscription(String, "/streamdeck/base_teleop_mode", self._base_mode_cb, 10)
        self.create_subscription(Bool, "/streamdeck/reset_config", self._reset_cb, 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        # self.create_subscription(Bool, "opensot/reset_complete", self._reset_complete_cb, 1)

        self.create_subscription(Bool, "opensot/home_done", self._home_done_cb, 10)
        self.create_subscription(String, "opensot/home_cmd", self._home_cmd_cb, 10)
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
                PointStamped, f"/vive/{side}/gripper", lambda m, s=side: self._gripper_cb(m, s), 10
            )
            self.create_subscription(
                PointStamped,
                f"/replay/{side}/gripper",
                lambda m, s=side: self._gripper_cb(m, s),
                10,
            )

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

        # --- PUBLISHERS ---
        self.pub_gripper_left = self.create_publisher(
            JointTrajectory, "/gripper_left_controller/joint_trajectory", 10
        )
        self.pub_gripper_right = self.create_publisher(
            JointTrajectory, "/gripper_right_controller/joint_trajectory", 10
        )
        self.pub_target_r = self.create_publisher(
            PoseStamped, "cartesian_interface/right/target_pose", 1
        )
        self.pub_target_l = self.create_publisher(
            PoseStamped, "cartesian_interface/left/target_pose", 1
        )
        self.pub_target_b = self.create_publisher(
            Twist, "cartesian_interface/base/target_twist", 10
        )
        self.pub_pause_opensot = self.create_publisher(Bool, "opensot/pause", 10)
        self.pub_home_cmd = self.create_publisher(String, "opensot/home_cmd", 10)

        self.srv_homes = {}
        for home_name in self.home_configs.keys():
            self.srv_homes[home_name] = self.create_service(
                Trigger,
                f"home_position/{home_name}",
                lambda req, res, name=home_name: self._home_service_cb(req, res, name),
            )

        self.gripper_state = {"left": "OPEN", "right": "OPEN"}
        self.gripper_btn_prev = {"left": 1.0, "right": 1.0}
        self.pose_synced = {"right": True, "left": False}

        for side in ["right", "left"]:
            self._send_gripper(side, self.gripper_open_pos)

        # --- RVIZ MARKER SERVER ---
        self.server = InteractiveMarkerServer(self, "six_dof_marker_server")
        self.menu_handler = MenuHandler()
        self.enable_entry = self.menu_handler.insert("Enable Task", callback=self._menu_cb)
        self.menu_handler.setCheckState(self.enable_entry, MenuHandler.CHECKED)
        self.menu_handler.insert("Reset", callback=self._menu_cb)

        self._wait_for_tf()
        self._init_marker("right")
        self._init_marker("left")

        # Control Loop (100Hz)
        self.create_timer(0.01, self._output_loop)
        self.get_logger().info("Cartesian Interface Node Initialized (Native Homing Forwarder)")

    def _home_done_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.get_logger().info("Homing finished! Disabling tasks; markers snap once TF settles.")

        # 1. Stop publishing commands right away.
        self.task_enabled = {"right": False, "left": False}
        self.menu_handler.setCheckState(self.enable_entry, MenuHandler.UNCHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        # 2. Clear cached teleop poses.
        for side in ["right", "left"]:
            self.vive_poses[side] = None
            self.replay_poses[side] = None
            self.pose_synced[side] = False

        # 3. Defer the marker snap. Snapping now reads the opensot/ TF mid-homing
        #    (RSP republishes at 50 Hz); a short delay lets it reach the final pose.
        if self.marker_reset_timer is not None:
            self.destroy_timer(self.marker_reset_timer)
        self.marker_reset_timer = self.create_timer(0.5, self._execute_delayed_marker_reset)

    def _osot(self, frame_id: str) -> str:
        clean_frame = frame_id.replace("opensot/", "").lstrip("/")
        return f"opensot/{clean_frame}"

    def _reset_complete_cb(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info("OpenSoT Reset Confirmed. Unmuting OpenSoT...")
            self.pub_pause_opensot.publish(Bool(data=False))
            if self.marker_reset_timer is not None:
                self.destroy_timer(self.marker_reset_timer)
            self.marker_reset_timer = self.create_timer(0.5, self._execute_delayed_marker_reset)

    def _begin_homing(self, target_name: str) -> None:
        """Stop commanding the arms + clear cached teleop poses.

        Runs from BOTH the home_position/<name> service and directly off
        /opensot/home_cmd, so homing triggered by a raw topic publish (not the
        service) still stops us feeding the solver a stale marker target.
        """
        self.get_logger().info(f"Homing initiated ({target_name}); disabling task output.")
        self.task_enabled = {"right": False, "left": False}
        self.menu_handler.setCheckState(self.enable_entry, MenuHandler.UNCHECKED)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

        # Drop any pending marker-reset from a previous home.
        if self.marker_reset_timer is not None:
            self.destroy_timer(self.marker_reset_timer)
            self.marker_reset_timer = None

        for side in ["right", "left"]:
            self.vive_poses[side] = None
            self.replay_poses[side] = None
            self.pose_synced[side] = False

    def _home_cmd_cb(self, msg: String) -> None:
        self._begin_homing(msg.data)

    def _home_service_cb(self, request, response, target_config_name) -> Trigger.Response:
        self._begin_homing(target_config_name)
        msg = String()
        msg.data = target_config_name
        self.pub_home_cmd.publish(msg)
        response.success = True
        response.message = f"Homing '{target_config_name}' commanded to OpenSoT native loop."
        return response

    def _mode_cb(self, msg: String) -> None:
        self._reset_cb(Bool(data=True))
        self.teleop_mode = msg.data
        if self.teleop_mode == "rviz":
            self._init_marker("right")
            self._init_marker("left")
        else:
            self.server.clear()
            self.server.applyChanges()

    def _base_mode_cb(self, msg: String) -> None:
        self._reset_cb(Bool(data=True))
        self.base_teleop_mode = msg.data

    def _reset_cb(self, msg: Bool) -> None:
        if msg.data:
            self.task_enabled = {"right": False, "left": False}
            self.menu_handler.setCheckState(self.enable_entry, MenuHandler.UNCHECKED)
            self.menu_handler.reApply(self.server)
            self.server.applyChanges()

    def _execute_delayed_marker_reset(self) -> None:
        # Snap the markers onto the end effectors -- but only once the opensot/ TF
        # has actually caught up to the post-homing pose (RSP republishes it at
        # 50 Hz). Retry every 0.2 s until the transform is fresh (< 0.2 s old).
        if self.marker_reset_timer is not None:
            self.destroy_timer(self.marker_reset_timer)
            self.marker_reset_timer = None

        poses, max_age, missing = {}, 0.0, False
        for side in ["right", "left"]:
            p, age = self._fk_transform(side)
            if p is None:
                missing = True
                break
            poses[side] = p
            if age is not None:
                max_age = max(max_age, age)

        if (missing or max_age > 0.2) and self._marker_reset_tries < 8:
            self._marker_reset_tries += 1
            self.marker_reset_timer = self.create_timer(0.2, self._execute_delayed_marker_reset)
            return

        self._marker_reset_tries = 0
        for side, p in poses.items():
            self.vive_poses[side] = None
            self.replay_poses[side] = None
            self.pose_synced[side] = False
            self.marker_poses[side] = p
            self._fk_cache[side] = p
            self.server.setPose(side, p)
            self.get_logger().info(
                f"Homing: {side} marker -> "
                f"({p.position.x:.3f}, {p.position.y:.3f}, {p.position.z:.3f}) "
                f"[tf age {max_age * 1000:.0f} ms]"
            )
        self.server.applyChanges()

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

    def _vive_trackpad_cb(self, source: str, side: str, axis: str, msg: PointStamped) -> None:
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
                self.gripper_closed_pos if desired_state == "CLOSED" else self.gripper_open_pos
            )
            self._send_gripper(side, target_pos)
        self.gripper_btn_prev[side] = msg.point.x

    def _send_gripper(self, side: str, pos: float) -> None:
        pub = self.pub_gripper_left if side == "left" else self.pub_gripper_right
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
        pub.publish(traj)

    def _wait_for_tf(self) -> None:
        target_frame = self._osot(self.frames["right"])
        base_frame = self._osot(self.frames["base_right"])
        while rclpy.ok():
            if self.tf_buffer.can_transform(base_frame, target_frame, rclpy.time.Time()):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

    def _fk_transform(self, side: str):
        """Latest opensot/ FK for `side` as (Pose, age_s), or (None, None) on a TF miss."""
        base_frame = self._osot(self.frames[f"base_{side}"])
        target_frame = self._osot(self.frames[side])
        try:
            t = self.tf_buffer.lookup_transform(base_frame, target_frame, rclpy.time.Time())
        except TransformException:
            return None, None
        p = Pose()
        p.position.x = t.transform.translation.x
        p.position.y = t.transform.translation.y
        p.position.z = t.transform.translation.z
        p.orientation = t.transform.rotation
        try:
            clk = self.get_clock()
            stamp = rclpy.time.Time.from_msg(t.header.stamp, clock_type=clk.clock_type)
            age = (clk.now() - stamp).nanoseconds / 1e9
        except (ValueError, TypeError):
            age = None
        return p, age

    def _get_fk_pose(self, side: str) -> Pose:
        base_frame = self._osot(self.frames[f"base_{side}"])
        target_frame = self._osot(self.frames[side])
        try:
            # Non-blocking: this runs in the 100 Hz timer, so we must not wait on
            # TF. _wait_for_tf() has already blocked until the tree is up.
            t = self.tf_buffer.lookup_transform(base_frame, target_frame, rclpy.time.Time())
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            self._fk_cache[side] = p
            return p
        except TransformException as e:
            cached = self._fk_cache.get(side)
            if cached is not None:
                return cached
            self.get_logger().warn(
                f"FK lookup {base_frame} <- {target_frame} failed and no cached pose: {e}",
                throttle_duration_sec=2.0,
            )
            p = Pose()
            p.orientation.w = 1.0
            return p

    def _get_visual_marker(self, task_link_name: str) -> Marker:
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 1.0, 0.0, 1.0)
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
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.5, 0.5, 0.5, 1.0)
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
            for mode in [InteractiveMarkerControl.MOVE_AXIS, InteractiveMarkerControl.ROTATE_AXIS]:
                mc = InteractiveMarkerControl()
                mc.orientation.w = 1.0
                setattr(mc.orientation, ax, 1.0)
                mc.interaction_mode = mode
                mc.name = (
                    f"rotate_{ax}" if mode == InteractiveMarkerControl.ROTATE_AXIS else f"move_{ax}"
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

    def _menu_cb(self, fb: InteractiveMarkerFeedback) -> None:
        name = fb.marker_name
        if fb.menu_entry_id == self.enable_entry:
            st = self.menu_handler.getCheckState(self.enable_entry)
            if st == MenuHandler.CHECKED:
                self.menu_handler.setCheckState(self.enable_entry, MenuHandler.UNCHECKED)
                self.task_enabled[name] = False
            else:
                self._reset_marker(name)
                self.menu_handler.setCheckState(self.enable_entry, MenuHandler.CHECKED)
                self.task_enabled[name] = True
        else:
            self._reset_marker(name)

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def _tf_replay(self, msg: PoseStamped, target_frame: str) -> PoseStamped | None:
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

    def _scale_twist(self, twist: Twist, lin_scale: float = 0.3, ang_scale: float = 0.3) -> Twist:
        scaled = Twist()
        scaled.linear.x = twist.linear.x * lin_scale
        scaled.linear.y = twist.linear.y * lin_scale
        scaled.angular.z = twist.angular.z * ang_scale
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
            target_pose = self.marker_poses[side]
            self.pose_synced[side] = True
        elif self.teleop_mode == "vive":
            target_msg = self.vive_poses[side]
        elif self.teleop_mode == "replay":
            target_msg = self._tf_replay(self.replay_poses[side], self.frames[f"base_{side}"])

        if target_msg is not None:
            now = self.get_clock().now()
            msg_time = rclpy.time.Time.from_msg(target_msg.header.stamp)
            age = (now - msg_time).nanoseconds / 1e9
            if age > 0.5:
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

            if not self.pose_synced[side]:
                if dist < 0.15:
                    self.pose_synced[side] = True
                else:
                    return

            msg = PoseStamped()
            msg.header.frame_id = self._osot(self.frames[f"base_{side}"])
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = target_pose
            pub.publish(msg)

    def _process_base_commands(self) -> None:
        raw_target_b = Twist()
        if self.base_teleop_mode == "joystick":
            raw_target_b = self._scale_twist(
                self.joy_twist, self.joy_scale_linear, self.joy_scale_angular
            )
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
