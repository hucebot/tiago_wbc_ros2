#!/usr/bin/env python3
"""Whole-body Cartesian controller for the (original) TIAGo Dual.

This node is the Dual counterpart of ``tiago_pro_opensot_node``. The Pro robot
is the one that gets most of the day-to-day attention, so this file is kept
deliberately close to it in structure: same ROS interface, same homing
handshake, same stack layout. The robot-specific bits (frames, capsule URDF
name, home-config set, camera frame, controller names) live in the ``ROBOT``
dict below so the two nodes can eventually be merged into one parametrised
node -- see TODO.md, section P2.

Differences from the Pro node that are intentional here:
  * joint <-> configuration-vector indices are resolved once from the model
    (``q_index_map`` / ``v_index_map`` from utils) instead of being hard-coded, so the
    off-by-N wheel bug in the Pro node cannot happen here;
  * the gaze task is opt-in (``enable_gaze`` parameter, default False) because
    the Dual capsule model does not necessarily expose a head-camera link;
  * self-collision pairs come from ``utils.collision_list`` (the hand-kept Dual
    list) with a JSON override if the config package ships one.
"""

import array
import copy
import json
import os
import time

import numpy as np

# OpenSoT / xbot2
import pyopensot as pysot
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from pyopensot.constraints.velocity import JointLimits, VelocityLimits
from pyopensot.tasks.velocity import Cartesian, Manipulability, Postural
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from xbot2_interface import pyaffine3, pyxbot2_collision
from xbot2_interface import pyxbot2_interface as xbi

try:
    from pyopensot.tasks.velocity import Gaze

    _HAS_GAZE = True
except ImportError:  # older pyopensot builds
    _HAS_GAZE = False

from pyopensot_collision.constraints.velocity import CollisionAvoidance

from tiago_control_node.utils import (
    EPS_REGULARISATION,
    ObstacleData,
    load_home_poses,
    q_index_map,
    v_index_map,
)
from tiago_control_node.utils import collision_list as DEFAULT_COLLISION_LIST

# --- Robot-specific configuration -------------------------------------------------
ROBOT = {
    "config_package": "tiago_dual_cartesio_config",
    "capsule_urdf": os.path.join("capsules", "urdf", "tiago_dual_capsules.urdf"),
    "collision_pairs_json": os.path.join(
        "capsules", "urdf", "tiago_dual_capsules_collision_pairs.json"
    ),
    "frame_right": "gripper_right_grasping_frame",
    "frame_left": "gripper_left_grasping_frame",
    "frame_base": "base_link",
    "frame_world": "world",
    "camera_frame": "head_front_camera_link",
    "default_home_key": "default",
    "joint_limit_margin": 0.025,  # fraction of range trimmed off each arm joint limit
    "controllers": [
        "arm_left_controller",
        "arm_right_controller",
        "head_controller",
        "torso_controller",
    ],
}


class TiagoOpenSoTNode(Node):
    def __init__(self):
        super().__init__("tiago_opensot_control")

        # --- Parameters ---
        self.declare_parameters(
            namespace="",
            parameters=[
                ("control_dt", 0.01),
                ("lambdas.gripper_right", 0.1),
                ("lambdas.gripper_left", 0.1),
                ("lambdas.postural", 0.08),
                ("lambdas.base", 0.1),
                ("enable_gaze", False),
                ("frames.right_gripper", ROBOT["frame_right"]),
                ("frames.left_gripper", ROBOT["frame_left"]),
                ("frames.base_link", ROBOT["frame_base"]),
                ("frames.world", ROBOT["frame_world"]),
                ("frames.camera", ROBOT["camera_frame"]),
                ("base_frames.right_arm_task", ROBOT["frame_base"]),
                ("base_frames.left_arm_task", ROBOT["frame_base"]),
                ("base_frames.base_task", ROBOT["frame_world"]),
            ],
        )

        self.dt = self.get_parameter("control_dt").value
        self.l_right = self.get_parameter("lambdas.gripper_right").value
        self.l_left = self.get_parameter("lambdas.gripper_left").value
        self.l_postural = self.get_parameter("lambdas.postural").value
        self.l_base = self.get_parameter("lambdas.base").value
        self.enable_gaze = self.get_parameter("enable_gaze").value and _HAS_GAZE

        self.frame_right = self.get_parameter("frames.right_gripper").value
        self.frame_left = self.get_parameter("frames.left_gripper").value
        self.frame_base = self.get_parameter("frames.base_link").value
        self.frame_world = self.get_parameter("frames.world").value
        self.frame_camera = self.get_parameter("frames.camera").value
        self.base_right_arm = self.get_parameter("base_frames.right_arm_task").value
        self.base_left_arm = self.get_parameter("base_frames.left_arm_task").value
        self.base_robot = self.get_parameter("base_frames.base_task").value

        # --- State ---
        self.target_right = None
        self.target_left = None
        self.target_base_twist = Twist()
        self.needs_reset = False
        self.enable_external_obstacle = False
        self.active_collisions = {}
        self.is_paused = False
        self.gaze_locked = True
        self.home_settle_until = 0.0  # ignore incoming arm targets until this time

        # Homing state machine
        self.homing_active = False
        self.is_currently_homing = False
        self.homing_target_q = {}
        self.homing_target_q_full = None
        self.homing_start_q = None
        self.homing_start_time = 0.0
        self.homing_duration = 0.5  # seconds of interpolation
        self.homing_settle = 2.0  # extra seconds allowed to converge
        self.homing_tol = 0.05  # rad RMS error that counts as "home"

        # --- Subscribers ---
        qos_state = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Bool, "/opensot/pause", self._pause_cb, 10)
        self.create_subscription(
            PoseStamped, "/cartesian_interface/right/target_pose", self._right_target_cb, 10
        )
        self.create_subscription(
            PoseStamped, "/cartesian_interface/left/target_pose", self._left_target_cb, 10
        )
        self.create_subscription(
            Twist, "/cartesian_interface/base/target_twist", self._base_target_cb, 10
        )
        self.create_subscription(Bool, "/streamdeck/reset_config", self._reset_cb, 10)
        self.create_subscription(
            MarkerArray, "/opensot/external_collisions", self._collision_scene_cb, 10
        )
        self.create_subscription(Bool, "/opensot/gaze_lock", self._gaze_lock_cb, qos_state)
        self.create_subscription(String, "/opensot/home_cmd", self._home_cmd_cb, 10)

        # --- Publishers ---
        self.joint_state_publisher = self.create_publisher(JointState, "/opensot/joint_states", 10)
        self.base_vel_publisher = self.create_publisher(Twist, "/opensot/base_velocity_command", 10)
        self.reset_ok_publisher = self.create_publisher(Bool, "/opensot/reset_complete", 1)
        self.home_done_pub = self.create_publisher(Bool, "/opensot/home_done", 10)
        self.collision_distances_publisher = self.create_publisher(
            Marker, "/opensot/viz/collision_distances", 10
        )
        self.active_collisions_publisher = self.create_publisher(
            MarkerArray, "/opensot/viz/active_collisions", 10
        )
        self.base_link_broadcaster = TransformBroadcaster(self)

        # --- Services ---
        self.enable_external_collision_service = self.create_service(
            SetBool, "enable_external_obstacle", self.handle_enable_external_collision
        )

        self.package_share_path = get_package_share_directory(ROBOT["config_package"])
        self.urdf = self._load_urdf()
        self.home_configs = load_home_poses("dual")

        if not self.enable_gaze:
            self.get_logger().info("Gaze task disabled (set 'enable_gaze' to turn it on).")
        self.get_logger().info("Tiago Dual OpenSoT control node initialized.")

    # --- URDF ------------------------------------------------------------------
    def _load_urdf(self) -> str:
        urdf_path = os.path.join(self.package_share_path, ROBOT["capsule_urdf"])
        try:
            with open(urdf_path) as f:
                return f.read()
        except OSError as e:
            raise RuntimeError(f"Could not load Dual capsule URDF at {urdf_path}: {e}") from e

    def _load_collision_pairs(self):
        """Return the set of link pairs to check for self-collision."""
        json_path = os.path.join(self.package_share_path, ROBOT["collision_pairs_json"])
        try:
            with open(json_path) as f:
                raw = json.load(f)["collision_list"]
            self.get_logger().info(f"Loaded {len(raw)} collision pairs from {json_path}")
            return {tuple(sorted(pair)) for pair in raw}
        except (OSError, KeyError, json.JSONDecodeError):
            self.get_logger().info(
                "No collision-pairs JSON in the config package; using utils.collision_list."
            )
            return {tuple(sorted(pair)) for pair in DEFAULT_COLLISION_LIST}

    # Joint <-> vector index maps live in utils (q_index_map / v_index_map).

    def _build_home_q(self, config_dict: dict) -> dict:
        jnt_map = {}
        for i, val in enumerate(config_dict.get("arm_left", [])):
            jnt_map[f"arm_left_{i + 1}_joint"] = val
        for i, val in enumerate(config_dict.get("arm_right", [])):
            jnt_map[f"arm_right_{i + 1}_joint"] = val
        if "torso" in config_dict:
            jnt_map["torso_lift_joint"] = config_dict["torso"][0]
        for i, val in enumerate(config_dict.get("head", [])):
            jnt_map[f"head_{i + 1}_joint"] = val
        return jnt_map

    def from_state_msg(self, msg, model) -> np.ndarray:
        q = np.zeros(model.getJointPosition().size)
        q[3:7] = [0.0, 0.0, 0.0, 1.0]  # identity quaternion for the floating base

        home_map = self._build_home_q(self.home_configs[ROBOT["default_home_key"]])
        ros_map = dict(zip(msg.name, msg.position, strict=False)) if msg else {}

        for name, i in q_index_map(model).items():
            if "wheel" in name:
                q[i] = 1.0  # (cos, sin) = (1, 0)
                continue
            if i >= len(q):
                continue
            if name in ros_map:
                q[i] = ros_map[name]
            elif name in home_map:
                q[i] = home_map[name]
        return q

    # --- Callbacks ---------------------------------------------------------------
    def _home_cmd_cb(self, msg: String):
        if msg.data in self.home_configs:
            self.get_logger().info(f"Received native homing command for: {msg.data}")
            self.homing_target_q = self._build_home_q(self.home_configs[msg.data])
            self.homing_active = True
        else:
            self.get_logger().warn(f"Unknown home config: {msg.data}")

    def _gaze_lock_cb(self, msg: Bool):
        self.gaze_locked = msg.data

    def _pause_cb(self, msg: Bool):
        self.is_paused = msg.data

    def _right_target_cb(self, msg: PoseStamped):
        if time.perf_counter() >= self.home_settle_until:
            self.target_right = msg

    def _left_target_cb(self, msg: PoseStamped):
        if time.perf_counter() >= self.home_settle_until:
            self.target_left = msg

    def _base_target_cb(self, msg: Twist):
        self.target_base_twist = msg

    def _reset_cb(self, msg: Bool):
        if msg.data:
            self.needs_reset = True
            self.reset_poses()

    def _collision_scene_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            obj_id = f"ext_{marker.ns}_{marker.id}"
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                if obj_id in self.active_collisions:
                    self.active_collisions[obj_id].status = "PENDING_DELETE"
            elif marker.action in (Marker.ADD, Marker.MODIFY):
                self.active_collisions[obj_id] = ObstacleData(marker=marker, status="PENDING_ADD")

    def handle_enable_external_collision(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        self.enable_external_obstacle = request.data
        response.success = True
        return response

    def reset_poses(self):
        self.target_right = None
        self.target_left = None
        self.target_base_twist = Twist()

    # --- Visualisation --------------------------------------------------------------
    def pub_collision_distances(self, collision_distance_points, current_time):
        marker = Marker()
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "opensot/world"
        marker.header.stamp = current_time
        marker.ns = "collision_distances"
        marker.id = 0
        marker.scale.x = 0.005
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.4, 0.6, 0.7, 0.8
        for pa, pb in collision_distance_points:
            marker.points.append(Point(x=pa[0], y=pa[1], z=pa[2]))
            marker.points.append(Point(x=pb[0], y=pb[1], z=pb[2]))
        self.collision_distances_publisher.publish(marker)

    def publish_active_obstacles(self, current_time):
        msg = MarkerArray()
        if self.enable_external_obstacle:
            for obs in self.active_collisions.values():
                if obs.status == "ACTIVE":
                    m = copy.deepcopy(obs.marker)
                    m.header.stamp = current_time
                    m.action = Marker.ADD
                    msg.markers.append(m)
        else:
            m = Marker()
            m.action = Marker.DELETEALL
            msg.markers.append(m)
        if msg.markers:
            self.active_collisions_publisher.publish(msg)

    # --- Output ------------------------------------------------------------------
    def pub_to_control_bridge(self, joint_state_msg, q, dq, qidx, vidx, wheel_names, tail_start):
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Continuous wheel joints are stored as (cos, sin) pairs -> recover the angle.
        for out_i, name in enumerate(wheel_names):
            ci = qidx[name]
            joint_state_msg.position[out_i] = float(np.arctan2(q[ci + 1], q[ci]))

        joint_state_msg.position[len(wheel_names) :] = array.array("d", q[tail_start:])
        joint_state_msg.velocity = array.array("d", [0.0] * len(joint_state_msg.position))

        # Torso gets a velocity feed-forward (it is slow and benefits from it).
        if "torso_lift_joint" in vidx:
            joint_state_msg.velocity[len(wheel_names)] = dq[vidx["torso_lift_joint"]] / self.dt

        self.joint_state_publisher.publish(joint_state_msg)

    def wait_for_initial_state(self, timeout=4.0):
        self.get_logger().info(f"Waiting for initial hardware state (timeout: {timeout}s)...")
        base_state = None

        def base_cb(msg):
            nonlocal base_state
            base_state = msg

        sub_base = self.create_subscription(JointState, "/joint_states", base_cb, 1)

        collected_refs = {}
        subs = []
        for ctrl in ROBOT["controllers"]:
            subs.append(
                self.create_subscription(
                    JointTrajectoryControllerState,
                    f"/{ctrl}/controller_state",
                    lambda msg, name=ctrl: collected_refs.update({name: msg}),
                    1,
                )
            )

        start_time = time.time()
        success = False
        while rclpy.ok():
            if base_state is not None and len(collected_refs) == len(ROBOT["controllers"]):
                success = True
                break
            if time.time() - start_time > timeout:
                self.get_logger().warn(
                    "Hardware synchronization timeout! Falling back to home config."
                )
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub_base)
        for s in subs:
            self.destroy_subscription(s)
        if not success:
            return None

        final_msg = JointState()
        final_msg.header = base_state.header
        final_msg.name = list(base_state.name)
        final_msg.position = list(base_state.position)
        name_to_idx = {n: i for i, n in enumerate(final_msg.name)}

        for state_msg in collected_refs.values():
            if not getattr(state_msg, "reference", None) or not state_msg.reference.positions:
                continue
            for i, joint_name in enumerate(state_msg.joint_names):
                if joint_name in name_to_idx:
                    final_msg.position[name_to_idx[joint_name]] = state_msg.reference.positions[i]
        return final_msg


def setup_opensot_stack(model: "xbi.ModelInterface2", node: TiagoOpenSoTNode):
    g_left = Cartesian("gripper_left_marker", model, node.frame_left, node.base_left_arm)
    g_left.setLambda(node.l_left)
    g_right = Cartesian("gripper_right_marker", model, node.frame_right, node.base_right_arm)
    g_right.setLambda(node.l_right)

    base = Cartesian("Cartesian_Base", model, node.frame_base, node.frame_world)
    base.rotateToLocal(True)
    base.setLambda(0.0)

    postural = Postural(model)
    postural.setLambda(node.l_postural)

    q_homing = Postural(model)
    q_homing.setWeight(0.0)

    manip_left = Manipulability(model, g_left)
    manip_right = Manipulability(model, g_right)

    tasks = {
        "left": g_left,
        "right": g_right,
        "postural": postural,
        "base": base,
        "manip_left": manip_left,
        "manip_right": manip_right,
        "q_homing": q_homing,
    }

    gaze = None
    if node.enable_gaze:
        try:
            gaze = Gaze("Gaze", model, node.frame_base, node.frame_camera)
            tasks["gaze"] = gaze
        except Exception as e:  # noqa: BLE001 - xbot2 raises plain exceptions
            node.get_logger().warn(f"Could not create Gaze task ({e}); continuing without it.")
            gaze = None

    # --- Joint / velocity limits (pad only the arm joints) ---
    # getJointLimits() is nv-sized -> index it in velocity space, not q space.
    qmin, qmax = model.getJointLimits()
    qmin_padded, qmax_padded = np.copy(qmin), np.copy(qmax)
    margin = ROBOT["joint_limit_margin"]
    for name, i in v_index_map(model).items():
        if ("arm_left" in name or "arm_right" in name) and i < len(qmax):
            rng = qmax[i] - qmin[i]
            qmax_padded[i] = qmax[i] - rng * margin
            qmin_padded[i] = qmin[i] + rng * margin

    qlims = JointLimits(model, qmax_padded, qmin_padded)
    dqlims = VelocityLimits(model, model.getVelocityLimits(), node.dt)

    base_con = Cartesian("Base_Con", model, node.frame_base, node.frame_world)
    base_con.setLambda(node.l_base)

    # --- Self-collision avoidance ---
    collision_avoidance = None
    try:
        collision_avoidance = CollisionAvoidance(model, max_pairs=1000, collision_urdf=node.urdf)
        collision_avoidance.setCollisionList(node._load_collision_pairs())
    except Exception as e:  # noqa: BLE001
        node.get_logger().warn(f"Collision avoidance disabled ({e}).")
        collision_avoidance = None

    # --- Assemble the stack ---
    top = g_left + g_right + base % [0, 1, 5] + q_homing
    if gaze is not None:
        top = top + gaze
    second = postural[6:] + 0.005 * manip_left + 0.005 * manip_right

    stack = (top / second) << qlims << dqlims
    if collision_avoidance is not None:
        stack = stack << collision_avoidance
    stack = stack << base_con % [2, 3, 4]

    solver = pysot.iHQP(stack, eps_regularisation=EPS_REGULARISATION)
    return solver, stack, tasks, collision_avoidance


def sync_external_collisions(node: TiagoOpenSoTNode, collision_avoidance):
    if collision_avoidance is None:
        return
    for obj_id, obs in list(node.active_collisions.items()):
        if obs.status == "PENDING_DELETE":
            collision_avoidance.setCollisionShapeActive(obj_id, False)
            del node.active_collisions[obj_id]
            continue

        if obs.status == "PENDING_ADD":
            m = obs.marker
            shape = None
            if m.type == Marker.CUBE:
                shape = pyxbot2_collision.shape.Box()
                shape.size = np.array([m.scale.x, m.scale.y, m.scale.z])
            elif m.type == Marker.SPHERE:
                shape = pyxbot2_collision.shape.Sphere()
                shape.radius = m.scale.x / 2.0
            elif m.type == Marker.CYLINDER:
                shape = pyxbot2_collision.shape.Cylinder()
                shape.radius = m.scale.x / 2.0
                shape.length = m.scale.z
            elif m.type == Marker.TRIANGLE_LIST:
                shape = pyxbot2_collision.shape.MeshRaw()
                shape.vertices = np.array([[p.x, p.y, p.z] for p in m.points])
                shape.triangles = np.arange(len(m.points), dtype=np.int32).reshape((-1, 3))
                shape.convex = True

            if shape is not None:
                w_T_c = pyaffine3.Affine3()
                w_T_c.translation = np.array(
                    [m.pose.position.x, m.pose.position.y, m.pose.position.z]
                )
                w_T_c.linear = R.from_quat(
                    [
                        m.pose.orientation.x,
                        m.pose.orientation.y,
                        m.pose.orientation.z,
                        m.pose.orientation.w,
                    ]
                ).as_matrix()
                collision_avoidance.addCollisionShape(obj_id, "world", shape, w_T_c, [])
                obs.status = "ACTIVE"

        if obs.status == "ACTIVE":
            collision_avoidance.setCollisionShapeActive(obj_id, node.enable_external_obstacle)


def _homing_rms_error(q, node, qidx):
    err = 0.0
    for name, target in node.homing_target_q.items():
        if name in qidx and qidx[name] < len(q):
            err += (q[qidx[name]] - target) ** 2
    return np.sqrt(err)


def _hold_cartesian(task, model, frame, base):
    """Pin a Cartesian task's reference to the frame's current pose, zero twist."""
    cur = model.getPose(frame, base)
    ref = task.getReference()[0]
    ref.translation = cur.translation
    ref.linear = cur.linear
    task.setReference(ref, np.zeros(6))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TiagoOpenSoTNode()
        model = xbi.ModelInterface2(node.urdf)

        q = node.from_state_msg(node.wait_for_initial_state(), model)
        model.setJointPosition(q)
        model.update()

        solver, stack, tasks, collision_avoidance = setup_opensot_stack(model, node)

        qidx = q_index_map(model)
        vidx = v_index_map(model)
        wheel_names = [n for n in model.getJointNames() if "wheel" in n]
        tail_start = min(i for n, i in qidx.items() if "wheel" not in n)

        msg = JointState()
        msg.name = model.getJointNames()[1:]
        msg.position = [0.0] * len(msg.name)

        w_T_b_tf = TransformStamped()
        w_T_b_tf.header.frame_id = "opensot/world"
        w_T_b_tf.child_frame_id = "opensot/base_footprint"

        # Safety latch: after this many consecutive solver failures, stop sending
        # joint commands (robot holds its last pose) until the solver recovers.
        SOLVER_FAIL_LIMIT = 20
        solver_fail_streak = 0

        while rclpy.ok():
            start = time.perf_counter()

            if node.needs_reset:
                node.reset_poses()
                q = node.from_state_msg(node.wait_for_initial_state(), model)
                node.needs_reset = False
                model.setJointPosition(q)
                model.update()
                for t in tasks.values():
                    if hasattr(t, "reset"):
                        t.reset()
                node.reset_ok_publisher.publish(Bool(data=True))

            model.setJointPosition(q)
            model.update()

            # Keep the gaze pointed at the right gripper while not homing.
            if "gaze" in tasks and not node.homing_active:
                tasks["gaze"].setGaze(model.getPose(node.frame_right, node.frame_base))

            # --- Native homing procedure ---
            # Steer q_homing, postural AND both arm Cartesian references to the
            # same interpolated config every tick: every task pulls the same way,
            # so there is no priority fight and no completion handoff.
            if node.homing_active:
                if not node.is_currently_homing:
                    node.is_currently_homing = True
                    node.get_logger().info("Starting native OpenSoT homing...")
                    node.target_base_twist = Twist()

                    node.homing_start_time = time.perf_counter()
                    node.homing_start_q = np.copy(q)
                    node.homing_target_q_full = np.copy(q)
                    for name, target in node.homing_target_q.items():
                        if name in qidx and qidx[name] < len(q):
                            node.homing_target_q_full[qidx[name]] = target

                    tasks["q_homing"].setWeight(0.5)
                    tasks["q_homing"].setLambda(0.1)
                    tasks["postural"].setLambda(0.1)
                    if "gaze" in tasks:
                        tasks["gaze"].setLambda(0.0)

                # Reject stale arm targets during homing and for 0.5 s after.
                node.home_settle_until = time.perf_counter() + 0.5
                node.target_right = None
                node.target_left = None

                elapsed_home = time.perf_counter() - node.homing_start_time
                s = float(np.clip(elapsed_home / node.homing_duration, 0.0, 1.0))
                q_ref = node.homing_start_q + s * (node.homing_target_q_full - node.homing_start_q)

                tasks["q_homing"].setReference(q_ref)
                tasks["postural"].setReference(q_ref)

                model.setJointPosition(q_ref)
                model.update()
                for tkey, frame, base in (
                    ("left", node.frame_left, node.base_left_arm),
                    ("right", node.frame_right, node.base_right_arm),
                ):
                    ee = model.getPose(frame, base)
                    ref = tasks[tkey].getReference()[0]
                    ref.translation = ee.translation
                    ref.linear = ee.linear
                    tasks[tkey].setReference(ref, np.zeros(6))
                model.setJointPosition(q)
                model.update()

                q_err = _homing_rms_error(q, node, qidx)
                converged = q_err < node.homing_tol
                timed_out = elapsed_home > (node.homing_duration + node.homing_settle)

                if s >= 1.0 and (converged or timed_out):
                    if converged:
                        node.get_logger().info(f"Homing complete! (Final error: {q_err:.3f})")
                    else:
                        node.get_logger().warn(
                            f"Homing timed out! Forcing completion. (Final error: {q_err:.3f})"
                        )

                    node.homing_active = False
                    node.is_currently_homing = False

                    # No handoff: every reference is already at the home config.
                    tasks["q_homing"].setWeight(0.0)
                    node.target_right = None
                    node.target_left = None
                    node.home_done_pub.publish(Bool(data=True))
            else:
                # Cartesian goals are only tracked when not homing.
                for target_msg, task, frame, base in (
                    (node.target_right, tasks["right"], node.frame_right, node.base_right_arm),
                    (node.target_left, tasks["left"], node.frame_left, node.base_left_arm),
                ):
                    if target_msg is not None:
                        p_ref = task.getReference()[0]
                        p_ref.translation = [
                            target_msg.pose.position.x,
                            target_msg.pose.position.y,
                            target_msg.pose.position.z,
                        ]
                        p_ref.linear = R.from_quat(
                            [
                                target_msg.pose.orientation.x,
                                target_msg.pose.orientation.y,
                                target_msg.pose.orientation.z,
                                target_msg.pose.orientation.w,
                            ]
                        ).as_matrix()
                        task.setReference(p_ref, np.zeros(6))
                    else:
                        # No target -> hold exactly here (reset() drifts in this build).
                        _hold_cartesian(task, model, frame, base)

            # Gaze weight / lambda (only when not homing).
            if "gaze" in tasks and not node.homing_active:
                gaze_dim = tasks["gaze"].getTaskSize()
                if node.gaze_locked:
                    tasks["gaze"].setLambda(0.0)
                    tasks["gaze"].setWeight(np.zeros((gaze_dim, gaze_dim)))
                else:
                    tasks["gaze"].setLambda(1.0)
                    tasks["gaze"].setWeight(np.eye(gaze_dim))

            # Base velocity command.
            v = node.target_base_twist
            tasks["base"].setVelocityLocalReference(
                np.array(
                    [v.linear.x * node.dt, v.linear.y * node.dt, 0, 0, 0, v.angular.z * node.dt]
                ).reshape(6, 1)
            )

            # Solve.
            sync_external_collisions(node, collision_avoidance)
            stack.update()

            dq = np.zeros(model.getNv())
            try:
                dq = solver.solve()
                if solver_fail_streak >= SOLVER_FAIL_LIMIT:
                    node.get_logger().warn("Solver recovered; resuming joint commands.")
                solver_fail_streak = 0
            except Exception as e:  # noqa: BLE001
                solver_fail_streak += 1
                node.get_logger().error(f"Solver fail: {e}", throttle_duration_sec=1.0)

            q = model.sum(q, dq)

            solver_halted = solver_fail_streak >= SOLVER_FAIL_LIMIT
            if solver_halted and solver_fail_streak == SOLVER_FAIL_LIMIT:
                node.get_logger().error(
                    f"Solver failed {SOLVER_FAIL_LIMIT}x in a row -- holding position, "
                    "not publishing joint commands until it recovers."
                )

            if not node.is_paused and not solver_halted:
                node.pub_to_control_bridge(msg, q, dq, qidx, vidx, wheel_names, tail_start)

            ts = node.get_clock().now().to_msg()
            w_T_b_tf.header.stamp = ts
            (
                w_T_b_tf.transform.translation.x,
                w_T_b_tf.transform.translation.y,
                w_T_b_tf.transform.translation.z,
            ) = q[0:3]
            (
                w_T_b_tf.transform.rotation.x,
                w_T_b_tf.transform.rotation.y,
                w_T_b_tf.transform.rotation.z,
                w_T_b_tf.transform.rotation.w,
            ) = q[3:7]
            node.base_link_broadcaster.sendTransform(w_T_b_tf)

            rclpy.spin_once(node, timeout_sec=0)

            if collision_avoidance is not None:
                node.pub_collision_distances(collision_avoidance.getOrderedWitnessPointVector(), ts)
            node.publish_active_obstacles(ts)

            elapsed = time.perf_counter() - start
            if elapsed < node.dt:
                time.sleep(node.dt - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
