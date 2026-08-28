#!/usr/bin/env python3
"""MuJoCo-based simulated hardware bridge for the Franka Panda - single-arm counterpart of
tiago_pro_mujoco_bridge/mujoco_sim_node.py, built for the WBC ablation described in project
memory "panda_wbc_ablation_goal": collect Panda data through the same kind of pipeline and
schema as TIAGo (panda_opensot_node.py plays the role of tiago_pro_opensot_node.py; this
node plays the role of mujoco_sim_node.py) so a policy trained on it is comparable to one
trained on TIAGo data.

Every timing fix found and fixed in mujoco_sim_node.py today is carried over here directly -
these were generic physics-loop/ROS-executor bugs, not TIAGo-specific, and the same classes
of bug would otherwise resurface on this robot too:
  1. Recording is paced by a fixed-cadence wall-clock accumulator (_maybe_log, in main()),
     NOT a ROS timer - a timer can only fire when spin_once() dispatches it, which lets a
     busy subscription (here, /opensot/joint_states) starve it out regardless of configured
     rate. "Fixed cadence" (next_log_time += log_period, not next_log_time = now +
     log_period) matters too: re-anchoring to whenever it last fired lets a single iteration
     of ordinary timing jitter permanently cost a recording opportunity - see main()'s
     _maybe_log for the full explanation.
  2. step_physics() re-applies the newest queued /opensot/joint_states command before EACH
     physics substep (between_substeps callback), not once per whole render iteration -
     otherwise this loop's own render rate (not OpenSoT's true ~100Hz publish rate) is what
     determines how often a command actually takes effect, silently coalescing/discarding
     whatever OpenSoT ticks arrived in between.
  3. The main loop's end-of-iteration pacing uses a sleep-then-busy-spin helper
     (_sleep_precise), not a plain time.sleep() - time.sleep() has no hard guarantee of
     waking up at the requested time, and under any real scheduler contention this
     routinely overshoots by several ms, which is what was capping achieved loop rate well
     below the configured fps even though compute itself had large headroom.
  4. EpisodeRecorder (imported from tiago_pro_mujoco_bridge, unmodified) already uses
     time.monotonic() for its internal timestamps, not time.time() - reused as-is here.

Single-arm specifics vs. mujoco_sim_node.py: one gripper (a tendon-coupled pair of fingers,
ctrl range 0-255, not two separate per-finger position actuators), one eef site
("ee_panda", not ee_left/ee_right - see robots/panda/urdf/panda.urdf's header comment for
why its pose exactly matches panda_opensot_node.py's Cartesian task frame), and an 8-D
action schema (pos(3)+quat(4)+gripper(1), not TIAGo's 16-D dual-arm layout) - see
get_log_entry() below. eef_right_pose (kept under the 'right' key, not a bare 'eef_pose',
for field-name parity with TIAGo's schema - matches pose_commander.py's own 'right'-only
convention for a single-arm robot) is the COMMANDED /cartesian_interface/right/target_pose,
matching mujoco_sim_node.py's own now-corrected convention (see that file's docstring) -
Panda is *also* going through OpenSoT-mediated tracking here, not a from-scratch IK, so the
same "commanded pose is what the pipeline's own recorded semantics need" reasoning applies
directly, unlike a from-scratch ManiSkill-facing node that would need to match ManiSkill's
own achieved-ground-truth tcp_pose convention instead.
"""
import time

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from tiago_pro_mujoco_bridge.episode_recorder import EpisodeRecorder

# Joint name -> MuJoCo position actuator name. Deliberately identical strings on both sides
# (unlike a robot where URDF/MuJoCo joint names differ) - robots/panda/urdf/panda.urdf was
# renamed specifically so this needs no translation table, mirroring how TIAGo's own
# JOINT_TO_MOTOR happens to have matching URDF/MuJoCo joint names too.
JOINT_TO_MOTOR = {f'joint{i}': f'actuator{i}' for i in range(1, 8)}

# Same values as panda_opensot_node.py's HOME_POSITIONS / panda.xml's own <keyframe
# name="home"> - kept in sync by hand across the three places that need it (MuJoCo has no
# way to import a shared Python config into its own XML keyframe).
HOME_POSITIONS = {
    'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': -1.57079,
    'joint5': 0.0, 'joint6': 1.57079, 'joint7': -0.7853,
}

# ctrl values for actuator8 (the tendon-coupled gripper actuator - see panda.xml's own
# comment: ctrlrange 0-255 remaps the underlying 0-0.04m per-finger travel). 255 = fully
# open (matches panda.xml's own "home" keyframe: ctrl=255 paired with qpos=0.04,0.04). 0 =
# commanding fully closed, which is what generates real grip force against an object in the
# way (same "deliberately overshoot the object's own width" reasoning as TIAGo's
# GRIPPER_CLOSED_POS, which also isn't the physically-fully-closed value) - not empirically
# verified against an actual grasp (no compiled pyopensot/xbot2_interface available outside
# the project's dev container), nudge if grip looks too tight/loose in the viewer.
GRIPPER_OPEN_CTRL = 255.0
GRIPPER_CLOSED_CTRL = 0.0

# How far straight up the target object gets parked while the robot is resetting - see
# mujoco_sim_node.py's identical constant/reasoning for why (avoid a stale pre-resync
# command sweeping the arm through the object's on-table spawn point).
OBJECT_PARK_Z_OFFSET = 1.5

HOME_POSITION_TOLERANCE_RAD = 0.01
HOME_VELOCITY_TOLERANCE_RAD_S = 0.01

# See mujoco_sim_node.py's identical constants for the full reasoning (both are carried
# over unchanged - generic physics-loop/ROS-executor properties, not TIAGo-specific).
MAX_SPINS_PER_STEP = 3
TIMING_LOG_PERIOD_SEC = 2.0


class PandaSimNode(Node):
    def __init__(self):
        super().__init__('panda_sim_node')

        self.declare_parameter('mujoco_xml_path',
            '/home/forest_ws/robots/panda/xmls/scene_panda.xml')
        self.declare_parameter('viewer', True)
        self.declare_parameter('fps', 90.0)
        self.declare_parameter('episode_log_fps', 90.0)
        self.declare_parameter('command_topic', '/opensot/joint_states')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('gripper_speed', 400.0)  # ctrl units/sec (0-255 range, not radians)
        self.declare_parameter('target_object_joint', 'cube_freejoint')
        self.declare_parameter('episode_log_path', '/tmp/panda_episodes/dataset.h5')
        self.declare_parameter('save_failed_episodes', False)
        self.declare_parameter('object_x_range', [0.50, 0.65])
        self.declare_parameter('object_y_range', [-0.20, -0.10])
        self.declare_parameter('base_frame', 'opensot/link0')
        # Same topic pose_commander.py's gripper publisher and panda_opensot_node.py's
        # target subscriber use for the 'right' side - see module docstring.
        self.declare_parameter('commanded_pose_topic', '/cartesian_interface/right/target_pose')
        self.declare_parameter('gripper_topic', '/mujoco_bridge/gripper_right/open')

        xml_path = self.get_parameter('mujoco_xml_path').value
        self.use_viewer = self.get_parameter('viewer').value
        self.fps = self.get_parameter('fps').value
        self.episode_log_fps = self.get_parameter('episode_log_fps').value
        if self.episode_log_fps > self.fps:
            self.get_logger().warn(
                f"episode_log_fps ({self.episode_log_fps}) > fps ({self.fps}) - the "
                "recording rate can't exceed the physics loop's own rate. Raise fps to match.")
        command_topic = self.get_parameter('command_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        target_object_joint = self.get_parameter('target_object_joint').value
        episode_log_path = self.get_parameter('episode_log_path').value
        save_failed_episodes = self.get_parameter('save_failed_episodes').value
        self.object_x_range = tuple(self.get_parameter('object_x_range').value)
        self.object_y_range = tuple(self.get_parameter('object_y_range').value)
        self.base_frame = self.get_parameter('base_frame').value
        commanded_pose_topic = self.get_parameter('commanded_pose_topic').value
        gripper_topic = self.get_parameter('gripper_topic').value
        self.episode_idx = 0

        self.get_logger().info(f"Loading MuJoCo model: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.steps_per_render = max(1, int(round((1.0 / self.fps) / self.model.opt.timestep)))

        self.qpos_adr = {}
        self.dof_adr = {}
        self.actuator_id = {}
        for joint_name, motor_name in JOINT_TO_MOTOR.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_name)
            if jid == -1 or aid == -1:
                self.get_logger().warn(f"'{joint_name}' / '{motor_name}' not found in MuJoCo model, skipping.")
                continue
            self.qpos_adr[joint_name] = self.model.jnt_qposadr[jid]
            self.dof_adr[joint_name] = self.model.jnt_dofadr[jid]
            self.actuator_id[joint_name] = aid

        self.joint_names = list(self.actuator_id.keys())
        self.target_positions = dict(HOME_POSITIONS)

        self.gripper_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'actuator8')
        if self.gripper_actuator_id == -1:
            self.get_logger().warn("Gripper actuator 'actuator8' not found in MuJoCo model.")
        self.gripper_status = 'open'
        self.current_gripper_ctrl = GRIPPER_OPEN_CTRL

        self.eef_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'ee_panda')
        if self.eef_site_id == -1:
            self.get_logger().warn("Site 'ee_panda' not found in MuJoCo model; eef ground-truth debug field disabled.")

        tjid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, target_object_joint)
        if tjid == -1:
            self.get_logger().warn(f"Joint '{target_object_joint}' not found; target_object_pose logging disabled.")
            self.target_object_qpos_adr = None
            self.target_object_dof_adr = None
        else:
            self.target_object_qpos_adr = self.model.jnt_qposadr[tjid]
            self.target_object_dof_adr = self.model.jnt_dofadr[tjid]
            self.object_rest_z = float(self.model.qpos0[self.target_object_qpos_adr + 2])
            self.object_rest_xy = (
                float(self.model.qpos0[self.target_object_qpos_adr]),
                float(self.model.qpos0[self.target_object_qpos_adr + 1]),
            )

        self.recorder = EpisodeRecorder(
            episode_log_path, save_failed_episodes, self.episode_log_fps, logger=self.get_logger())

        self._reset_robot_home()

        self.command_sub = self.create_subscription(
            JointState, command_topic, self._command_cb, 10)

        # Commanded (not achieved) eef pose - see module docstring. Populated from the same
        # topic panda_opensot_node.py's Cartesian task subscribes to, so this is exactly
        # what got commanded, not MuJoCo's own achieved ground truth.
        self.commanded_pose = None
        self.create_subscription(PoseStamped, commanded_pose_topic, self._commanded_pose_cb, 10)

        self.joint_state_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.target_object_pose_pub = self.create_publisher(PoseStamped, '/mujoco_bridge/target_object_pose', 10)
        self.settled_at_home_pub = self.create_publisher(Bool, '/mujoco_bridge/settled_at_home', 10)

        self.create_subscription(Bool, gripper_topic, self._gripper_cmd_cb, 10)

        self.create_service(Trigger, '/mujoco_bridge/sim/reset_robot_home', self._reset_robot_home_cb)
        self.create_service(Trigger, '/mujoco_bridge/sim/randomize_object_pose', self._randomize_object_pose_cb)
        self.create_service(SetBool, '/mujoco_bridge/sim/save_episode_log', self._save_episode_log_cb)

        # mujoco_sim_node.py has an equivalent _apply_sim_reset_timeout_override() here that
        # tells tiago_pro_opensot_control to skip its real-robot hardware-wait timeout on
        # reset. panda_opensot_node.py has no such parameter - its reset path (see that
        # file's docstring) never had a real-robot bootstrap wait to skip in the first
        # place - so there's deliberately nothing to call here.

        self.get_logger().info(
            f"Panda MuJoCo sim node ready. Driving {len(self.joint_names)} joints "
            f"from '{command_topic}', publishing state on '{joint_states_topic}', "
            f"recording at {self.episode_log_fps}Hz.")

    def _gripper_cmd_cb(self, msg: Bool):
        self.gripper_status = 'open' if msg.data else 'close'

    def _commanded_pose_cb(self, msg: PoseStamped):
        p, q = msg.pose.position, msg.pose.orientation
        self.commanded_pose = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def _sim_eef_pos_quatwxyz(self):
        """MuJoCo's own live ground-truth end-effector pose - sim-only debug signal, NOT
        what eef_right_pose logs (see module docstring)."""
        if self.eef_site_id == -1:
            return None, None
        pos = self.data.site_xpos[self.eef_site_id].copy()
        quat_wxyz = np.zeros(4)
        mujoco.mju_mat2Quat(quat_wxyz, self.data.site_xmat[self.eef_site_id])
        return pos, quat_wxyz

    def _reset_robot_home(self):
        mujoco.mj_resetData(self.model, self.data)
        for joint_name, pos in HOME_POSITIONS.items():
            if joint_name not in self.qpos_adr:
                continue
            self.data.qpos[self.qpos_adr[joint_name]] = pos
            self.data.ctrl[self.actuator_id[joint_name]] = pos
        if self.gripper_actuator_id != -1:
            self.data.ctrl[self.gripper_actuator_id] = GRIPPER_OPEN_CTRL
        self.current_gripper_ctrl = GRIPPER_OPEN_CTRL
        self.gripper_status = 'open'
        self.target_positions = dict(HOME_POSITIONS)
        if self.target_object_qpos_adr is not None:
            adr = self.target_object_qpos_adr
            x, y = self.object_rest_xy
            self.data.qpos[adr:adr + 3] = [x, y, self.object_rest_z + OBJECT_PARK_Z_OFFSET]
            self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
            if self.target_object_dof_adr is not None:
                self.data.qvel[self.target_object_dof_adr:self.target_object_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _reset_robot_home_cb(self, request, response):
        self.recorder.pause()
        self._reset_robot_home()
        response.success = True
        response.message = "robot reset to home"
        return response

    def _randomize_object_pose(self):
        if self.target_object_qpos_adr is None:
            return
        adr = self.target_object_qpos_adr
        x = float(np.random.uniform(*self.object_x_range))
        y = float(np.random.uniform(*self.object_y_range))
        self.data.qpos[adr:adr + 3] = [x, y, self.object_rest_z]
        self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
        if self.target_object_dof_adr is not None:
            self.data.qvel[self.target_object_dof_adr:self.target_object_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _randomize_object_pose_cb(self, request, response):
        self._randomize_object_pose()
        self.recorder.resume()
        response.success = True
        response.message = "object respawned"
        return response

    def _save_episode_log_cb(self, request, response):
        try:
            saved_path = self.recorder.save_and_clear(success=request.data, attempt_index=self.episode_idx)
            response.success = True
            response.message = saved_path or "no steps logged for this episode"
        except Exception as exc:
            self.get_logger().error(f"Failed to save episode log: {exc!r}")
            self.recorder.log_buffer = []
            self.recorder._timestamps = []
            response.success = False
            response.message = f"save failed: {exc!r}"
        self.episode_idx += 1
        return response

    def actuate_gripper(self):
        """Ramps the gripper smoothly towards open/closed - same shape as
        mujoco_sim_node.py's actuate_gripper(), but a single tendon-coupled actuator
        (ctrl units 0-255) instead of two independent per-finger position actuators."""
        if self.gripper_actuator_id == -1:
            return
        target = GRIPPER_OPEN_CTRL if self.gripper_status == 'open' else GRIPPER_CLOSED_CTRL
        dt = self.model.opt.timestep * self.steps_per_render
        current = self.current_gripper_ctrl
        step = self.gripper_speed * dt
        if current < target:
            current = min(current + step, target)
        elif current > target:
            current = max(current - step, target)
        self.current_gripper_ctrl = current
        self.data.ctrl[self.gripper_actuator_id] = current

    def _command_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_positions:
                self.target_positions[name] = pos

    def apply_targets(self):
        for joint_name, aid in self.actuator_id.items():
            self.data.ctrl[aid] = self.target_positions[joint_name]

    def step_physics(self, between_substeps=None):
        """See mujoco_sim_node.py's identical method for the full reasoning - re-applying
        the newest queued command before EACH substep, not once per render iteration,
        keeps pace with OpenSoT's own ~100Hz cadence regardless of this loop's own rate."""
        for _ in range(self.steps_per_render):
            if between_substeps is not None:
                between_substeps()
            mujoco.mj_step(self.model, self.data)

    def get_log_entry(self) -> dict:
        """Single-arm, 8-D action schema - see module docstring for why eef_right_pose is
        the COMMANDED pose (matches mujoco_sim_node.py's now-corrected convention) and why
        this is kept under a 'right' key (field-name parity with TIAGo's schema) rather than
        a bare, differently-named field."""
        obs = {}
        pos, quat_wxyz = self._sim_eef_pos_quatwxyz()
        if pos is not None:
            obs['eef_right_pos'] = pos
            obs['eef_right_rot'] = quat_wxyz  # (w,x,y,z) MuJoCo ground truth, sim-only, debug only

        if self.commanded_pose is not None:
            action_pose = self.commanded_pose
        elif pos is not None:
            action_pose = np.concatenate([pos, quat_wxyz[[1, 2, 3, 0]]])
        else:
            action_pose = np.zeros(7)
        obs['eef_right_pose'] = action_pose

        obs['joint_pos_opensot'] = np.array([self.target_positions[n] for n in self.joint_names])
        obs['joint_pos_real'] = np.array([self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        obs['joint_vel_real'] = np.array([self.data.qvel[self.dof_adr[n]] for n in self.joint_names])

        if self.target_object_qpos_adr is not None:
            adr = self.target_object_qpos_adr
            x, y, z, qw, qx, qy, qz = self.data.qpos[adr:adr + 7]
            obs['target_object_pose'] = np.array([x, y, z, qx, qy, qz, qw])

        # 1.0 = closed, 0.0 = open - same convention as TIAGo's gripper action encoding.
        gripper_val = 1.0 if self.gripper_status == 'close' else 0.0
        actions = np.concatenate([action_pose, [gripper_val]]).astype(np.float32)

        return {'actions': actions, 'obs': obs}

    def _log_step_cb(self):
        self.recorder.record(self.get_log_entry())

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.data.qpos[self.qpos_adr[n]] for n in self.joint_names]
        msg.velocity = [self.data.qvel[self.dof_adr[n]] for n in self.joint_names]
        self.joint_state_pub.publish(msg)

    def publish_target_object_pose(self):
        if self.target_object_qpos_adr is None:
            return
        adr = self.target_object_qpos_adr
        x, y, z, qw, qx, qy, qz = self.data.qpos[adr:adr + 7]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = float(qx), float(qy), float(qz), float(qw)
        self.target_object_pose_pub.publish(msg)

    def _is_settled_at_home(self) -> bool:
        for joint_name, home_pos in HOME_POSITIONS.items():
            if joint_name not in self.qpos_adr:
                continue
            pos_err = abs(self.data.qpos[self.qpos_adr[joint_name]] - home_pos)
            vel = abs(self.data.qvel[self.dof_adr[joint_name]])
            if pos_err > HOME_POSITION_TOLERANCE_RAD or vel > HOME_VELOCITY_TOLERANCE_RAD_S:
                return False
        return True

    def publish_settled_at_home(self):
        self.settled_at_home_pub.publish(Bool(data=self._is_settled_at_home()))


def main(args=None):
    rclpy.init(args=args)
    node = PandaSimNode()

    viewer_ctx = mujoco.viewer.launch_passive(node.model, node.data) if node.use_viewer else None

    log_period = 1.0 / node.episode_log_fps
    next_log_time = time.perf_counter()

    def _maybe_log(now):
        """See mujoco_sim_node.py's identical function for the full reasoning - fixed
        cadence (+= log_period), not re-anchored to whenever it last fired."""
        nonlocal next_log_time
        if now >= next_log_time:
            node._log_step_cb()
            next_log_time += log_period
            if next_log_time < now - log_period:
                next_log_time = now

    def _substep_update():
        rclpy.spin_once(node, timeout_sec=0)
        node.apply_targets()

    def _step():
        node.actuate_gripper()
        t0 = time.perf_counter()
        node.step_physics(between_substeps=_substep_update)
        t1 = time.perf_counter()
        for _ in range(MAX_SPINS_PER_STEP):
            rclpy.spin_once(node, timeout_sec=0)
        t2 = time.perf_counter()
        return (t1 - t0), (t2 - t1)

    timing = {
        'physics': 0.0, 'spin': 0.0, 'viewer_sync': 0.0, 'publish': 0.0, 'total': 0.0,
        'sleep_requested': 0.0, 'sleep_actual': 0.0, 'loop_period': 0.0,
    }
    timing_iters = 0
    last_timing_log = time.perf_counter()
    prev_iter_start = None

    def _sleep_precise(duration):
        """See mujoco_sim_node.py's identical function - sleeps most of the way, then
        busy-spins the last ~1.5ms, since time.sleep() has no hard wake-time guarantee."""
        end = time.perf_counter() + duration
        coarse = duration - 0.0015
        if coarse > 0:
            time.sleep(coarse)
        while time.perf_counter() < end:
            pass

    try:
        while rclpy.ok() and (viewer_ctx is None or viewer_ctx.is_running()):
            start = time.perf_counter()
            if prev_iter_start is not None:
                timing['loop_period'] += start - prev_iter_start
            prev_iter_start = start

            if viewer_ctx is not None:
                with viewer_ctx.lock():
                    physics_dt, spin_dt = _step()
                    _maybe_log(time.perf_counter())
                t_sync0 = time.perf_counter()
                viewer_ctx.sync()
                viewer_sync_dt = time.perf_counter() - t_sync0
            else:
                physics_dt, spin_dt = _step()
                _maybe_log(time.perf_counter())
                viewer_sync_dt = 0.0

            t_pub0 = time.perf_counter()
            node.publish_joint_state()
            node.publish_target_object_pose()
            node.publish_settled_at_home()
            publish_dt = time.perf_counter() - t_pub0

            elapsed = time.perf_counter() - start

            sleep_time = max(0.0, (1.0 / node.fps) - elapsed)
            if sleep_time > 0:
                t_sleep0 = time.perf_counter()
                _sleep_precise(sleep_time)
                sleep_actual = time.perf_counter() - t_sleep0
            else:
                sleep_actual = 0.0

            timing['physics'] += physics_dt
            timing['spin'] += spin_dt
            timing['viewer_sync'] += viewer_sync_dt
            timing['publish'] += publish_dt
            timing['total'] += elapsed
            timing['sleep_requested'] += sleep_time
            timing['sleep_actual'] += sleep_actual
            timing_iters += 1
            now = time.perf_counter()
            if now - last_timing_log >= TIMING_LOG_PERIOD_SEC:
                n = timing_iters
                budget_ms = 1000.0 / node.fps
                node.get_logger().info(
                    f"main-loop timing (avg over {n} iters, "
                    f"{n / timing['loop_period']:.1f}Hz achieved, budget {budget_ms:.2f}ms @ fps={node.fps}): "
                    f"physics={1000 * timing['physics'] / n:.2f}ms "
                    f"spin={1000 * timing['spin'] / n:.2f}ms "
                    f"viewer_sync={1000 * timing['viewer_sync'] / n:.2f}ms "
                    f"publish={1000 * timing['publish'] / n:.2f}ms "
                    f"compute_total={1000 * timing['total'] / n:.2f}ms "
                    f"sleep_requested={1000 * timing['sleep_requested'] / n:.2f}ms "
                    f"sleep_actual={1000 * timing['sleep_actual'] / n:.2f}ms "
                    f"loop_period={1000 * timing['loop_period'] / n:.2f}ms")
                timing = {k: 0.0 for k in timing}
                timing_iters = 0
                last_timing_log = now
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        node.recorder.save_and_clear(success=False, attempt_index=node.episode_idx)
        if viewer_ctx is not None:
            viewer_ctx.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
