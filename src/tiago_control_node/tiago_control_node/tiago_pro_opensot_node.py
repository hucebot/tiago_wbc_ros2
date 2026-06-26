import os
import sys
import time
import copy
import json
import array
import numpy as np
from scipy.spatial.transform import Rotation as R
from tiago_control_node.utils import tiago_pro_home_config as home_config, ObstacleData

# ROS 2 Interfaces
import rclpy
import tf2_geometry_msgs  # CRITICAL: Registers geometry_msgs transformations with TF2 natively
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
from pyopensot.tasks.velocity import Postural, Cartesian, Manipulability, Gaze
from pyopensot_collision.constraints.velocity import CollisionAvoidance


class TiagoOpenSoTNode(Node):
    def __init__(self):
        super().__init__('tiago_opensot_control')

        # --- Parameters ---
        param_defaults = [
            ('control_dt', 0.01),
            ('lambdas.gripper_right', 0.03),
            ('lambdas.gripper_left', 0.03),
            ('lambdas.postural', 0.01),
            ('lambdas.base', 0.1),
            ('frames.right_gripper', "gripper_right_grasping_link"),
            ('frames.left_gripper', "gripper_left_grasping_link"),
            ('frames.base_link', "base_link"),
            ('frames.world', "world"),
            ('base_frames.right_arm_task', "base_link"),
            ('base_frames.left_arm_task', "base_link"),
            ('base_frames.base_task', "world")
        ]
        self.declare_parameters(namespace='', parameters=param_defaults)
        self.get_logger().info("Parameters declared with defaults.")

        self.dt = self.get_parameter('control_dt').value
        self.l_right = self.get_parameter('lambdas.gripper_right').value
        self.l_left = self.get_parameter('lambdas.gripper_left').value
        self.l_postural = self.get_parameter('lambdas.postural').value
        self.l_base = self.get_parameter('lambdas.base').value

        # --- Frames ---
        self.frame_right = self.get_parameter('frames.right_gripper').value
        self.frame_left = self.get_parameter('frames.left_gripper').value
        self.frame_base = self.get_parameter('frames.base_link').value
        self.frame_world = self.get_parameter('frames.world').value
        self.base_right_arm = self.get_parameter('base_frames.right_arm_task').value
        self.base_left_arm = self.get_parameter('base_frames.left_arm_task').value
        self.base_robot = self.get_parameter('base_frames.base_task').value

        # --- State Variables ---
        self.target_right = None
        self.target_left = None
        self.target_base_twist = Twist()
        self.needs_reset = False
        self.enable_external_obstacle = False
        self.active_collisions = {}
        self.is_paused = False

        # --- Subscribers ---
        self.create_subscription(Bool, '/opensot/pause', self._pause_cb, 10)
        self.create_subscription(PoseStamped, '/cartesian_interface/right/target_pose', self._right_target_cb, 10)
        self.create_subscription(PoseStamped, '/cartesian_interface/left/target_pose', self._left_target_cb, 10)
        self.create_subscription(Twist, '/cartesian_interface/base/target_twist', self._base_target_cb, 10)
        self.create_subscription(Bool, '/streamdeck/reset_config', self._reset_cb, 10)
        self.create_subscription(MarkerArray, '/opensot/external_collisions', self._collision_scene_cb, 10)

        # --- Publishers ---
        self.joint_state_publisher = self.create_publisher(JointState, '/opensot/joint_states', 10)
        self.base_vel_publisher = self.create_publisher(Twist, '/opensot/base_velocity_command', 10)
        self.reset_ok_publisher = self.create_publisher(Bool, '/opensot/reset_complete', 1)
        self.collision_distances_publisher = self.create_publisher(Marker, '/opensot/viz/collision_distances', 10)
        self.active_collisions_publisher = self.create_publisher(MarkerArray, '/opensot/viz/active_collisions', 10)
        self.base_link_broadcaster = TransformBroadcaster(self)

        # --- Services ---
        self.enable_external_collision_service = self.create_service(
            SetBool, 'enable_external_obstacle', self.handle_enable_external_collision
        )

        # Load Robot URDF
        self.package_share_path = get_package_share_directory('tiago_dual_cartesio_config')
        self.urdf = self._load_urdf()
        self.get_logger().info("Tiago OpenSoT Control Node initialized successfully.")

    # --- CALLBACKS ---
    def _pause_cb(self, msg: Bool):
        self.is_paused = msg.data
        state = "PAUSED" if self.is_paused else "RESUMED"
        self.get_logger().info(f"OpenSoT execution {state}.")

    def _right_target_cb(self, msg: PoseStamped):
        self.target_right = msg

    def _left_target_cb(self, msg: PoseStamped):
        self.target_left = msg

    def _base_target_cb(self, msg: Twist):
        self.target_base_twist = msg

    def _reset_cb(self, msg: Bool):
        if msg.data:
            self.needs_reset = True
            self.reset_poses()

    def _collision_scene_cb(self, msg: MarkerArray):
        if not hasattr(self, 'active_collisions'):
            self.active_collisions = {}

        for marker in msg.markers:
            # Add 'ext_' prefix to avoid overlapping with robot URDF link names
            obj_id = f"ext_{marker.ns}_{marker.id}"

            if marker.action in [Marker.DELETE, Marker.DELETEALL]:
                if obj_id in self.active_collisions:
                    self.active_collisions[obj_id].status = "PENDING_DELETE"
            elif marker.action in [Marker.ADD, Marker.MODIFY]:
                self.active_collisions[obj_id] = ObstacleData(marker=marker, status="PENDING_ADD")

    # --- SERVICE HANDLERS ---
    def handle_enable_external_collision(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.enable_external_obstacle = request.data
        self.get_logger().info(f"External collisions globally {'enabled' if request.data else 'disabled'}.")
        response.success = True
        return response

    # --- PUBLISHERS ---
    def pub_collision_distances(self, collision_distance_points, current_time):
        marker = Marker()
        marker.pose.orientation.w = 1.0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.header.frame_id = "opensot/world"
        marker.header.stamp = current_time
        marker.ns = "collision_distances"
        marker.id = 0
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.4
        marker.color.g = 0.6
        marker.color.b = 0.7
        marker.color.a = 0.8

        for pa, pb in collision_distance_points:
            marker.points.append(Point(x=pa[0], y=pa[1], z=pa[2]))
            marker.points.append(Point(x=pb[0], y=pb[1], z=pb[2]))

        self.collision_distances_publisher.publish(marker)

    def pub_to_control_bridge(self, joint_state_msg, q, dq):
        """Translates OpenSoT floating base states into actionable ROS 2 commands."""
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # OpenSoT maps the floating base to quaternion/SE3 states; extract the planar angles
        joint_state_msg.position[0] = np.arctan2(np.sin(q[8]), np.cos(q[7]))
        joint_state_msg.position[1] = np.arctan2(np.sin(q[10]), np.cos(q[9]))
        joint_state_msg.position[2] = np.arctan2(np.sin(q[12]), np.cos(q[11]))
        joint_state_msg.position[3] = np.arctan2(np.sin(q[14]), np.cos(q[13]))
        joint_state_msg.position[4:] = array.array('d', q[15:])

        joint_state_msg.velocity = array.array('d', [0.0] * len(joint_state_msg.position))
        joint_state_msg.velocity[4] = dq[10] / self.dt

        # Publish hardware control state
        self.joint_state_publisher.publish(joint_state_msg)

    def publish_active_obstacles(self, current_time):
        """Echoes the active constraints back to RViz for visual confirmation."""
        msg = MarkerArray()

        if self.enable_external_obstacle:
            for obj_id, obs in self.active_collisions.items():
                if obs.status == "ACTIVE":
                    m = copy.deepcopy(obs.marker)
                    m.header.stamp = current_time
                    m.action = Marker.ADD
                    msg.markers.append(m)
        else:
            # Clear RViz screen if obstacles are disabled
            m = Marker()
            m.action = Marker.DELETEALL
            msg.markers.append(m)

        if msg.markers:
            self.active_collisions_publisher.publish(msg)

    # --- UTILS ---
    def _load_urdf(self) -> str:
        """Loads the Tiago Pro URDF from the installed package."""
        try:
            urdf_path = os.path.join(self.package_share_path, "capsules", "urdf", "tiago_pro_capsules.urdf")
            with open(urdf_path, 'r') as f:
                return f.read()
        except Exception as e:
            self.get_logger().fatal(f"Could not load Pro URDF: {e}")
            sys.exit(1)

    def from_state_msg(self, msg, model):
        """Sets the OpenSoT model configuration based on the hardware's real state."""
        q = copy.copy(home_config)

        # OpenSoT local frame starts at [0,0,0] - opensot/world handles the map offset
        q[0:3] = [0.0, 0.0, 0.0]
        q[3:7] = [0.0, 0.0, 0.0, 1.0]

        if msg is None:
            return q

        # Floating base accounts for indices 0-6; hardware joints start at 7
        idx = 7
        ros_map = dict(zip(msg.name, msg.position))

        for name in model.getJointNames():
            if name == 'reference':
                continue
            if "wheel" in name:
                idx += 2
            else:
                if name in ros_map and idx < len(q):
                    q[idx] = ros_map[name]
                idx += 1
        return q

    def wait_for_initial_state(self, timeout=1.0):
        """Waits for the initial state of the robot hardware to synchronize the solver."""
        self.get_logger().info(f"Waiting for initial hardware state (timeout: {timeout}s)...")

        base_state = None
        def base_cb(msg): nonlocal base_state; base_state = msg
        sub_base = self.create_subscription(JointState, '/joint_states', base_cb, 1)

        target_controllers = ['arm_left_controller', 'arm_right_controller', 'head_controller', 'torso_controller']
        collected_refs = {}

        subs = []
        def make_cb(name): return lambda msg: collected_refs.update({name: msg})
        for ctrl in target_controllers:
            subs.append(self.create_subscription(
                JointTrajectoryControllerState, f'/{ctrl}/controller_state', make_cb(ctrl), 1))

        start_time = time.time()
        success = False

        while rclpy.ok():
            if base_state is not None and len(collected_refs) == len(target_controllers):
                success = True
                break

            if time.time() - start_time > timeout:
                self.get_logger().warn("Hardware synchronization timeout! Falling back to home_config.")
                break

            rclpy.spin_once(self, timeout_sec=0.1)

        # Cleanup subscriptions
        self.destroy_subscription(sub_base)
        for s in subs:
            self.destroy_subscription(s)

        if not success:
            return None

        # Build composite initialization state
        final_msg = JointState()
        final_msg.header = base_state.header
        final_msg.name = list(base_state.name)
        final_msg.position = list(base_state.position)
        name_to_idx = {n: i for i, n in enumerate(final_msg.name)}

        for _, state_msg in collected_refs.items():
            if not hasattr(state_msg, 'reference') or not state_msg.reference.positions:
                continue
            for i, joint_name in enumerate(state_msg.joint_names):
                if joint_name in name_to_idx:
                    final_msg.position[name_to_idx[joint_name]] = state_msg.reference.positions[i]

        self.get_logger().info("Initial hardware state synchronized.")
        return final_msg

    def reset_poses(self):
        """Clears the cached target poses and twists."""
        self.target_right = None
        self.target_left = None
        self.target_base_twist = Twist()

def setup_opensot_stack(model: xbi.ModelInterface2, node: TiagoOpenSoTNode):
    """Builds and returns the OpenSoT Task/Constraint dictionaries."""

    # --- Primary Tasks ---
    g_left = Cartesian("gripper_left_marker", model, node.frame_left, node.base_left_arm)
    g_left.setLambda(node.l_left)

    g_right = Cartesian("gripper_right_marker", model, node.frame_right, node.base_right_arm)
    g_right.setLambda(node.l_right)

    base = Cartesian("Cartesian_Base", model, node.frame_base, node.frame_world)
    base.rotateToLocal(True)
    base.setLambda(0.0) # Ignore KP term, rely purely on velocity reference

    # --- Secondary Tasks ---
    postural = Postural(model)
    postural.setLambda(node.l_postural)

    manip_left = Manipulability(model, g_left)
    manip_right = Manipulability(model, g_right)
    gaze = Gaze("Gaze", model, "base_link", "head_front_camera_link")

    # --- Constraints ---
    qmin, qmax = model.getJointLimits()
    qlims = JointLimits(model, qmax, qmin)
    dqlims = VelocityLimits(model, model.getVelocityLimits(), node.dt)

    base_con = Cartesian("Base_Con", model, node.frame_base, node.frame_world)
    base_con.setLambda(node.l_base)

    # Setup specific collision pairs
    collision_pairs_path = os.path.join(
        node.package_share_path, "capsules", "urdf", "tiago_pro_capsules_collision_pairs.json"
    )
    collision_avoidance = CollisionAvoidance(model, max_pairs=1000, collision_urdf=node.urdf)

    with open(collision_pairs_path, 'r') as f:
        pro_collision_json = json.load(f)

    pro_collision_list = [(linkA, linkB) for pair in pro_collision_json["collision_list"] for linkA, linkB in [sorted(pair)]]
    collision_avoidance.setCollisionList(set(pro_collision_list))

    # --- Stack of Tasks ---
    stack = ((g_left + g_right + base % [0, 1, 5]) /
             (postural[6:] + 0.05 * manip_left + 0.05 * manip_right)) \
             << qlims << dqlims << collision_avoidance << base_con % [2, 3, 4]

    tasks = {
        "left": g_left,
        "right": g_right,
        "postural": postural,
        "base": base,
        "manip_left": manip_left,
        "manip_right": manip_right,
        "gaze": gaze
    }

    return pysot.iHQP(stack, eps_regularisation=1e10), stack, tasks, collision_avoidance

def sync_external_collisions(node: TiagoOpenSoTNode, collision_avoidance: CollisionAvoidance):
    """Processes the active_collisions dictionary and dynamically updates the QP solver constraints."""
    for obj_id, obs in list(node.active_collisions.items()):

        if obs.status == "PENDING_DELETE":
            collision_avoidance.setCollisionShapeActive(obj_id, False)
            del node.active_collisions[obj_id]
            continue

        elif obs.status == "PENDING_ADD":
            shape = None
            m = obs.marker

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

            if shape:
                w_T_c = pyaffine3.Affine3()
                w_T_c.translation = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
                w_T_c.linear = R.from_quat([
                    m.pose.orientation.x, m.pose.orientation.y,
                    m.pose.orientation.z, m.pose.orientation.w
                ]).as_matrix()

                collision_avoidance.addCollisionShape(obj_id, "world", shape, w_T_c, [])
                obs.status = "ACTIVE"

        if obs.status == "ACTIVE":
            collision_avoidance.setCollisionShapeActive(obj_id, node.enable_external_obstacle)

def main(args=None):
    rclpy.init(args=args)
    node = TiagoOpenSoTNode()
    model = xbi.ModelInterface2(node.urdf)

    # Initialize physical robot state
    init_msg = node.wait_for_initial_state()
    q = node.from_state_msg(init_msg, model)
    model.setJointPosition(q)
    model.update()

    solver, stack, tasks, collision_avoidance = setup_opensot_stack(model, node)

    # ROS 2 JointState configuration
    msg = JointState()
    msg.name = model.getJointNames()[1:]  # Skip floating "reference" joint
    msg.position = [0.0] * len(msg.name)

    # TF configuration
    w_T_b_tf = TransformStamped()
    w_T_b_tf.header.frame_id, w_T_b_tf.child_frame_id = "opensot/world", "opensot/base_footprint"

    try:
        while rclpy.ok():
            start = time.perf_counter()

            # Handle hard solver resets (e.g., from StreamDeck)
            if node.needs_reset:
                node.get_logger().info("RESETTING OpenSoT Model...")
                node.reset_poses()
                q = node.from_state_msg(node.wait_for_initial_state(), model)
                node.needs_reset = False

                model.setJointPosition(q)
                model.update()

                for t in tasks.values():
                    if hasattr(t, 'reset'):
                        t.reset()

                q_baseline = np.copy(q)
                node.reset_ok_publisher.publish(Bool(data=True))

            model.setJointPosition(q)
            model.update()

            # Gaze Task
            T = model.getPose("gripper_right_grasping_link", "base_link")
            tasks["gaze"].setGaze(T)

            # Cartesian Trajectory Commands
            for target_msg, task in [(node.target_right, tasks['right']), (node.target_left, tasks['left'])]:
                if target_msg is not None:
                    p_ref = task.getReference()[0]
                    p_ref.translation = [target_msg.pose.position.x, target_msg.pose.position.y, target_msg.pose.position.z]
                    p_ref.linear = R.from_quat([
                        target_msg.pose.orientation.x, target_msg.pose.orientation.y,
                        target_msg.pose.orientation.z, target_msg.pose.orientation.w
                    ]).as_matrix()
                    task.setReference(p_ref, np.zeros(6))
                else:
                    task.reset()

            # Base Commands
            v = node.target_base_twist
            tasks['base'].setVelocityLocalReference(np.array([
                v.linear.x * node.dt, v.linear.y * node.dt,
                0, 0, 0, v.angular.z * node.dt
            ]).reshape(6, 1))

            # Execution
            sync_external_collisions(node, collision_avoidance)
            stack.update()

            dq = np.zeros(model.getNv())
            try:
                dq = solver.solve()
            except Exception as e:
                node.get_logger().error(f"Solver fail: {e}", throttle_duration_sec=1.0)

            # Numerical Integration
            q = model.sum(q, dq)

            if not node.is_paused:
                node.pub_to_control_bridge(msg, q, dq)

            # Hardware TF Broadcaster
            ts = node.get_clock().now().to_msg()
            w_T_b_tf.header.stamp = ts
            w_T_b_tf.transform.translation.x, w_T_b_tf.transform.translation.y, w_T_b_tf.transform.translation.z = q[0:3]
            w_T_b_tf.transform.rotation.x, w_T_b_tf.transform.rotation.y, w_T_b_tf.transform.rotation.z, w_T_b_tf.transform.rotation.w = q[3:7]
            node.base_link_broadcaster.sendTransform(w_T_b_tf)

            rclpy.spin_once(node, timeout_sec=0)

            # Visualizations
            node.pub_collision_distances(collision_avoidance.getOrderedWitnessPointVector(), ts)
            node.publish_active_obstacles(ts)

            # Loop Rate Regulation
            elapsed = time.perf_counter() - start
            if elapsed < node.dt:
                time.sleep(node.dt - elapsed)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()