#!/usr/bin/env python3
"""MuJoCo-based simulated hardware bridge for Tiago Pro.

Stands in for the real robot on the tiago_pro_opensot_node / cartesian_interface_node
pipeline: publishes /joint_states from a live MuJoCo simulation and drives the
simulated torso + arm actuators from the OpenSoT solver's /opensot/joint_states output.

This is the ONLY node allowed to touch MuJoCo's qpos/ctrl (or the passive viewer) - it
owns the model, the data, and the physics loop outright. Anything that needs to reset the
robot, respawn the object, or save an episode's log does so by calling one of the three
services below, rather than reaching into MuJoCo state directly - see
episode_orchestrator_node.py, which is the thing that actually calls them in sequence
during a reset (this node deliberately doesn't know about episode/reset *sequencing*,
just how to do each individual step):
  - /mujoco_bridge/sim/reset_robot_home (Trigger): puts the robot back at its home
    posture and pauses recording.
  - /mujoco_bridge/sim/randomize_object_pose (Trigger): respawns the target object at a
    new random table position and resumes recording. Deliberately separate from the
    above - the object is only safe to move once the robot is confirmed settled at home
    (else MuJoCo's contact solver can find them overlapping and fling both apart).
  - /mujoco_bridge/sim/save_episode_log (SetBool, request.data=success): saves (or
    discards, per save_failed_episodes) the buffered episode.
  - /mujoco_bridge/sim/set_object_pose (PoseStamped, topic not service): teleports the
    object to an exact pose, bypassing object_x_range/object_y_range - for replaying a
    recorded episode against the object pose it actually saw (obs/target_object_pose),
    not a fresh random spawn.
  - /mujoco_bridge/sim/set_joint_state (JointState, topic not service): teleports named
    arm/torso joints straight to given positions, bypassing OpenSoT and the position-servo
    actuators - for tiago_replay.py's --ground-truth mode (frame-perfect obs/joint_pos_real
    playback, no tracking controller in the loop to diverge from the recording).

The tiago-pro-mujoco XML is fixed-base and headless (no wheel or head joints), so
base and head commands from OpenSoT are simply not applied here.

Recording is paced by a wall-clock accumulator in main()'s loop (NOT a ROS timer - see
that loop for why), so it can't run faster than episode_log_fps, and also can't run faster
than the loop's own true achieved rate. episode_log_fps must not exceed fps: __init__ warns
loudly if that's misconfigured, and the accumulator degrades gracefully to the loop's real
rate rather than falling arbitrarily far behind it.
"""
import os
import time
import array

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from tiago_control_node.utils import tiago_pro_home_config
from tiago_pro_mujoco_bridge.episode_recorder import EpisodeRecorder

# Joint name -> MuJoCo position actuator name (only the DoFs this MuJoCo model actuates)
JOINT_TO_MOTOR = {
    'torso_lift_joint': 'torso_lift_motor',
    **{f'arm_left_{i}_joint': f'arm_left_{i}_motor' for i in range(1, 8)},
    **{f'arm_right_{i}_joint': f'arm_right_{i}_motor' for i in range(1, 8)},
}

# Measured empirically in MuJoCo: 0.0 rad -> ~102mm fingertip gap (open), 0.9 rad -> ~7mm gap
# (closed, unobstructed). This is the OPPOSITE of tiago_pro_sim.py's comment - verified
# directly against this XML's linkage.
#
# GRIPPER_CLOSED_POS used to just be that fully-closed 0.9 - fine for an empty gripper, but
# with a ~40mm cube between the fingers, the position servo (kp=5, actuatorfrcrange +-8 on
# the driven finger joints) kept commanding a gap 33mm smaller than what's physically there,
# so it pressed into the object for the entire "closed" duration instead of stopping at
# contact - which is what was actually causing the "slipping" (deep interpenetration, not a
# friction problem). Retargeted to ~34mm gap: some excess-close beyond the object's own width
# is still needed to generate real grip force (that's how a position-controlled parallel
# gripper squeezes at all), just not 33mm of it. The 102mm/7mm mapping is an empirical
# 2-point measurement, assumed roughly linear here to back out a angle for a specific gap -
# this hasn't been reverified against the real gap once a 40mm object is actually in the
# way, so nudge it in the viewer if the grip still looks too tight or too loose.
GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSED_POS = 0.55

# tiago_pro_home_config layout: [floating_base(7), wheels(8), torso(1), arm_left(7), arm_right(7), head(2)]
HOME_POSITIONS = {
    'torso_lift_joint': tiago_pro_home_config[15],
    **{f'arm_left_{i}_joint': tiago_pro_home_config[15 + i] for i in range(1, 8)},
    **{f'arm_right_{i}_joint': tiago_pro_home_config[22 + i] for i in range(1, 8)},
}

# How close each torso/arm joint's qpos/qvel must be to HOME_POSITIONS (position) and zero
# (velocity) to count as "settled at home" - see _is_settled_at_home(). _reset_robot_home()
# writes qpos/qvel directly, so these are already exactly met the instant it runs - this
# check exists for the window AFTER that, while tiago_pro_opensot_node is still resyncing:
# if a stale (pre-reset) /opensot/joint_states message lands before that resync completes,
# apply_targets() would momentarily pull a joint away from home again. episode_orchestrator_node
# polls /mujoco_bridge/settled_at_home (published below) before it trusts the reset is really
# done, instead of just trusting /opensot/reset_complete alone.
HOME_POSITION_TOLERANCE_RAD = 0.01
HOME_VELOCITY_TOLERANCE_RAD_S = 0.01

# How far straight up (from its normal resting height) the target object gets parked while
# the robot is resetting - see _reset_robot_home()'s use of this. Well clear of the arm's
# reachable workspace, so a transient stale-target sweep (the same risk HOME_POSITION_
# TOLERANCE_RAD/settled_at_home exist for) can never clip it, unlike its normal on-table
# spawn point which sits right in the arm's path.
OBJECT_PARK_Z_OFFSET = 1.5

# Extra rclpy.spin_once(timeout_sec=0) calls per render iteration, ON TOP OF the one
# already interleaved before each physics substep in step_physics() (see its docstring).
# Those per-substep calls handle the busy ~100Hz /opensot/joint_states stream; this extra
# pass is only for everything else sharing the same executor (gripper toggles, resets,
# services) - low-rate/on-demand, so a handful of calls is plenty. Each call is cheap
# (returns immediately if nothing is ready) - a single spin_once() call dispatches AT MOST
# ONE ready callback, which is what let a busy subscription starve out the (now removed)
# recording timer before; keeping this small but >1 avoids reintroducing that same kind of
# starvation for these lower-rate callbacks.
MAX_SPINS_PER_STEP = 3

# How often (seconds) main() logs a breakdown of where main-loop time is actually going -
# see TIMING KEYS below. Purely observational (adds a few perf_counter() calls per
# iteration, no effect on control/recording) - exists because "fps is lower than
# requested" doesn't say WHY on its own, and guessing further without a measurement wastes
# time better spent just looking at the actual breakdown.
TIMING_LOG_PERIOD_SEC = 2.0


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        self.declare_parameter('mujoco_xml_path',
            '/home/forest_ws/robots/pal_tiago_pro/xmls/scene_tiago_pro.xml')
        self.declare_parameter('viewer', True)
        self.declare_parameter('fps', 90.0)
        # How often a step gets appended to the episode log - decoupled from the physics/
        # render loop's fps (see module docstring). Matches the real Tiago controller's
        # rate by default, since this is what Dont-Be-Brave/timid trains against; change
        # this, not fps, to change the recorded data's sample rate.
        self.declare_parameter('episode_log_fps', 90.0)
        self.declare_parameter('command_topic', '/opensot/joint_states')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('gripper_speed', 0.8)
        self.declare_parameter('target_object_joint', 'cube_freejoint')
        self.declare_parameter('episode_log_path', '/tmp/tiago_pro_episodes/dataset.h5')
        self.declare_parameter('save_failed_episodes', False)
        # Table-frame xy range the object is respawned into on /mujoco_bridge/sim/randomize_object_pose.
        # Kept small and centered on the cube's XML spawn point (0.6, -0.15) - the right
        # side of the table, in the right arm's workspace (see tasks/pick_place_basket.py's
        # PLAN, which then carries it to the basket on the table's far/left side); widen
        # once you've confirmed the corners are still reachable for your grasp approach.
        self.declare_parameter('object_x_range', [0.50, 0.65])
        self.declare_parameter('object_y_range', [-0.20, -0.10])
        # Frame the published target-object pose is expressed in. This MuJoCo scene is
        # fixed-base with the robot's base_link coincident with the MuJoCo world origin,
        # so raw world-frame qpos doubles as the base_link-frame pose the arm tasks expect
        # (see pose_commander.py, which consumes this topic directly as base_link-frame xyz).
        self.declare_parameter('base_frame', 'opensot/base_link')

        xml_path = self.get_parameter('mujoco_xml_path').value
        self.use_viewer = self.get_parameter('viewer').value
        self.fps = self.get_parameter('fps').value
        self.episode_log_fps = self.get_parameter('episode_log_fps').value
        if self.episode_log_fps > self.fps:
            self.get_logger().warn(
                f"episode_log_fps ({self.episode_log_fps}) > fps ({self.fps}) - the recording "
                "timer can't fire faster than the main loop spins ROS, so actual recording "
                "rate will be capped at fps, not episode_log_fps. Raise fps to match.")
        command_topic = self.get_parameter('command_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        self.gripper_speed = self.get_parameter('gripper_speed').value
        target_object_joint = self.get_parameter('target_object_joint').value
        episode_log_path = self.get_parameter('episode_log_path').value
        save_failed_episodes = self.get_parameter('save_failed_episodes').value
        self.object_x_range = tuple(self.get_parameter('object_x_range').value)
        self.object_y_range = tuple(self.get_parameter('object_y_range').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.episode_idx = 0  # total reset attempts, success or not - see _save_episode_log_cb

        self.get_logger().info(f"Loading MuJoCo model: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.steps_per_render = max(1, int(round((1.0 / self.fps) / self.model.opt.timestep)))

        # Resolve joint/actuator addresses once
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

        # Gripper fingers are actuated directly (not driven by /opensot/joint_states)
        self.gripper_actuator_id = {}
        for side in ('left', 'right'):
            for finger in ('left', 'right'):
                name = f'gripper_{side}_finger_{finger}'
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                if aid == -1:
                    self.get_logger().warn(f"Actuator '{name}' not found in MuJoCo model, skipping.")
                    continue
                self.gripper_actuator_id[name] = aid
        self.gripper_status = {'left': 'open', 'right': 'open'}
        self.current_gripper_pos = {'left': GRIPPER_OPEN_POS, 'right': GRIPPER_OPEN_POS}

        # End-effector sites and the logged target object (the cube), for get_log_entry()
        self.eef_site_id = {}
        for side in ('left', 'right'):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f'ee_{side}')
            if sid == -1:
                self.get_logger().warn(f"Site 'ee_{side}' not found in MuJoCo model; eef logging for {side} disabled.")
            self.eef_site_id[side] = sid

        tjid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, target_object_joint)
        if tjid == -1:
            self.get_logger().warn(f"Joint '{target_object_joint}' not found; target_object_pose logging disabled.")
            self.target_object_qpos_adr = None
            self.target_object_dof_adr = None
        else:
            self.target_object_qpos_adr = self.model.jnt_qposadr[tjid]
            self.target_object_dof_adr = self.model.jnt_dofadr[tjid]
            # The object's XML-defined resting height, read straight from the model's own
            # reference configuration (qpos0) rather than from a live MjData snapshot after
            # some reset - this is what randomize_object_pose() places the object at, so it
            # needs to be the true table-rest height regardless of what _reset_robot_home()
            # does with the object in the meantime (see OBJECT_PARK_Z_OFFSET below).
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

        # eef_{side}_pose (both here and in `actions`, see get_log_entry()) is logged as
        # this COMMANDED pose, not MuJoCo's achieved ground truth - matches what the real
        # robot pipeline already does (it has no ground-truth "current achieved pose" signal
        # to use instead, so it echoes the commanded value there too - see get_log_entry()'s
        # docstring). Logging the achieved pose instead used to seem like free extra
        # accuracy sim has and real doesn't, but it silently reintroduces OpenSoT's own
        # tracking lag on replay: a script that republishes a recorded "achieved" pose as a
        # NEW target has to converge to it all over again, so by the time a row's gripper
        # event fires (tied to the ORIGINAL recording's timing, where that row's arm was
        # already there), the replayed arm hasn't caught up yet - confirmed directly via
        # tiago_replay.py, which closes the gripper on air. A policy trained on that
        # achieved-pose signal would hit the exact same lag at inference, for the same
        # reason - so this needs to be the commanded pose for both sim and real, not just
        # for real.
        self.commanded_pose = {'left': None, 'right': None}
        for side in ('left', 'right'):
            self.create_subscription(
                PoseStamped, f'/cartesian_interface/{side}/target_pose',
                lambda msg, s=side: self._commanded_pose_cb(s, msg), 10)

        self.joint_state_pub = self.create_publisher(
            JointState, joint_states_topic, 10)
        self.target_object_pose_pub = self.create_publisher(
            PoseStamped, '/mujoco_bridge/target_object_pose', 10)
        self.settled_at_home_pub = self.create_publisher(Bool, '/mujoco_bridge/settled_at_home', 10)

        for side in ('left', 'right'):
            self.create_subscription(
                Bool, f'/mujoco_bridge/gripper_{side}/open',
                lambda msg, s=side: self._gripper_cmd_cb(s, msg), 10)

        self.create_service(Trigger, '/mujoco_bridge/sim/reset_robot_home', self._reset_robot_home_cb)
        self.create_service(Trigger, '/mujoco_bridge/sim/randomize_object_pose', self._randomize_object_pose_cb)
        self.create_service(SetBool, '/mujoco_bridge/sim/save_episode_log', self._save_episode_log_cb)
        self.create_subscription(
            PoseStamped, '/mujoco_bridge/sim/set_object_pose', self._set_object_pose_cb, 10)
        self.create_subscription(
            JointState, '/mujoco_bridge/sim/set_joint_state', self._set_joint_state_cb, 10)

        self._apply_sim_reset_timeout_override()

        self.get_logger().info(
            f"Tiago Pro MuJoCo sim node ready. Driving {len(self.joint_names)} joints "
            f"from '{command_topic}', publishing state on '{joint_states_topic}', "
            f"recording at {self.episode_log_fps}Hz.")

    def _apply_sim_reset_timeout_override(self):
        """Best-effort: tells the already-running tiago_pro_opensot_control node to skip its
        ~1s wait for real ros2_control hardware topics that never exist in this sim (see
        tiago_pro_opensot_node.py's reset_hardware_timeout_sec param). Purely a speed
        optimization for resets - if that node isn't up yet or the call fails, resets just
        stay at the real-robot-default ~1s wait; nothing else breaks."""
        cli = self.create_client(SetParameters, '/tiago_pro_opensot_control/set_parameters')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                "/tiago_pro_opensot_control/set_parameters not available - resets will use "
                "the default ~1s hardware-wait timeout instead of the sim-optimized one.")
            return
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='reset_hardware_timeout_sec',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.0),
        )]
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        result = future.result()
        if result is not None and result.results and result.results[0].successful:
            self.get_logger().info("tiago_pro_opensot_control reset_hardware_timeout_sec set to 0.0 (sim mode).")
        else:
            self.get_logger().warn(
                "Failed to set reset_hardware_timeout_sec on tiago_pro_opensot_control - "
                "resets will use the default ~1s hardware-wait timeout.")

    def _gripper_cmd_cb(self, side, msg: Bool):
        self.gripper_status[side] = 'open' if msg.data else 'close'

    def _commanded_pose_cb(self, side, msg: PoseStamped):
        """Latest /cartesian_interface/{side}/target_pose - see get_log_entry()/the
        comment on self.commanded_pose in __init__ for why this, not MuJoCo's own ground
        truth, is what gets logged as eef_{side}_pose. This scene is fixed-base with
        base_link coincident with the MuJoCo world origin, so msg.pose is used directly, no
        transform needed (same assumption target_object_pose/set_object_pose rely on)."""
        p, q = msg.pose.position, msg.pose.orientation
        self.commanded_pose[side] = np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def _sim_eef_pos_quatwxyz(self, side):
        """MuJoCo's own live ground-truth end-effector pose - position and (w,x,y,z)
        quaternion, or (None, None) if this model has no 'ee_{side}' site. Sim-only debug
        signal (obs/eef_{side}_pos/_rot) - NOT what eef_{side}_pose/actions log, see
        get_log_entry()."""
        sid = self.eef_site_id[side]
        if sid == -1:
            return None, None
        pos = self.data.site_xpos[sid].copy()
        quat_wxyz = np.zeros(4)
        mujoco.mju_mat2Quat(quat_wxyz, self.data.site_xmat[sid])
        return pos, quat_wxyz

    def _reset_robot_home(self):
        """Pure MuJoCo state reset - no recorder/pause side effects, so this can be shared
        between __init__ (before the recorder's timer even exists) and the service handler
        below (which does pause recording around it)."""
        mujoco.mj_resetData(self.model, self.data)
        for joint_name, pos in HOME_POSITIONS.items():
            if joint_name not in self.qpos_adr:
                continue
            self.data.qpos[self.qpos_adr[joint_name]] = pos
            self.data.ctrl[self.actuator_id[joint_name]] = pos
        for name, aid in self.gripper_actuator_id.items():
            self.data.ctrl[aid] = GRIPPER_OPEN_POS
        self.current_gripper_pos = {'left': GRIPPER_OPEN_POS, 'right': GRIPPER_OPEN_POS}
        self.gripper_status = {'left': 'open', 'right': 'open'}
        # apply_targets() overwrites data.ctrl from this dict every step - reset it too,
        # otherwise the very next control loop iteration immediately re-applies whatever
        # stale (non-home) joint targets OpenSoT last published, undoing the reset above.
        self.target_positions = dict(HOME_POSITIONS)
        # mj_resetData above already put the object back on the table at its default XML
        # spawn point - right in the arm's path. That's a problem specifically because the
        # robot ISN'T actually settled yet at this point (episode_orchestrator_node still
        # has to wait for OpenSoT to resync and confirm /mujoco_bridge/settled_at_home) - if
        # a stale pre-reset command sweeps the arm through that spot in the meantime, it
        # clips the object ("kicks it"). Whisk the object straight up out of the arm's
        # reachable workspace here instead, and leave it there - randomize_object_pose()
        # is what actually places it back on the table, and that's only ever called once
        # the robot is confirmed settled (see episode_orchestrator_node.py's end_episode
        # sequencing), so the object is never on the table while the robot might still move.
        if self.target_object_qpos_adr is not None:
            adr = self.target_object_qpos_adr
            x, y = self.object_rest_xy
            self.data.qpos[adr:adr + 3] = [x, y, self.object_rest_z + OBJECT_PARK_Z_OFFSET]
            self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]
            if self.target_object_dof_adr is not None:
                self.data.qvel[self.target_object_dof_adr:self.target_object_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _reset_robot_home_cb(self, request, response):
        # Redundant with save_and_clear() now also pausing (episode_orchestrator_node.py
        # always calls save_episode_log before this) - kept as defense in depth for any
        # other caller of this service that skips straight to a reset without saving first.
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
        self.data.qpos[adr + 3:adr + 7] = [1.0, 0.0, 0.0, 0.0]  # upright, no rotation
        if self.target_object_dof_adr is not None:
            self.data.qvel[self.target_object_dof_adr:self.target_object_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _randomize_object_pose_cb(self, request, response):
        self._randomize_object_pose()
        self.recorder.resume()
        response.success = True
        response.message = "object respawned"
        return response

    def _set_object_pose_cb(self, msg: PoseStamped):
        """Teleports the target object to an explicit pose - for tiago_replay.py, which
        needs the object sitting exactly where it was for the episode being replayed
        (recorded per-step in obs/target_object_pose), not wherever
        _randomize_object_pose() last happened to drop it. Bypasses the object_x_range/
        object_y_range clamp on purpose: this is "put it back exactly here", not a fresh
        randomized spawn."""
        if self.target_object_qpos_adr is None:
            self.get_logger().warn("set_object_pose: target_object_joint not configured, ignoring.")
            return
        adr = self.target_object_qpos_adr
        p, q = msg.pose.position, msg.pose.orientation
        self.data.qpos[adr:adr + 3] = [p.x, p.y, p.z]
        self.data.qpos[adr + 3:adr + 7] = [q.w, q.x, q.y, q.z]
        if self.target_object_dof_adr is not None:
            self.data.qvel[self.target_object_dof_adr:self.target_object_dof_adr + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)

    def _save_episode_log_cb(self, request, response):
        # An HDF5 write failure here (disk full, a stale in-memory demo counter colliding
        # with what's already in the file, ...) must not take the whole sim node down with
        # it - that would kill physics/the viewer/every other episode along with it, not
        # just this one save. Log it, drop the buffered episode, and keep running.
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

    def actuate_gripper(self, side, status):
        """Ramps one gripper smoothly towards open/closed. Call once per render frame."""
        if side not in ('left', 'right') or status not in ('open', 'close'):
            return

        target = GRIPPER_OPEN_POS if status == 'open' else GRIPPER_CLOSED_POS
        dt = self.model.opt.timestep * self.steps_per_render
        current = self.current_gripper_pos[side]

        if current < target:
            current = min(current + self.gripper_speed * dt, target)
        elif current > target:
            current = max(current - self.gripper_speed * dt, target)

        self.current_gripper_pos[side] = current
        for finger in ('left', 'right'):
            aid = self.gripper_actuator_id.get(f'gripper_{side}_finger_{finger}')
            if aid is not None:
                self.data.ctrl[aid] = current

    def actuate_grippers(self):
        for side in ('left', 'right'):
            self.actuate_gripper(side, self.gripper_status[side])

    def _command_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.target_positions:
                self.target_positions[name] = pos

    def _set_joint_state_cb(self, msg: JointState):
        """Teleports arm/torso joints straight to given positions, bypassing OpenSoT and the
        position-servo actuators entirely - for tiago_replay.py's --ground-truth mode, which
        wants a frame-perfect visual reproduction of obs/joint_pos_real with no controller
        in the loop to diverge from it.

        Also updates target_positions (apply_targets()'s ctrl source), not just qpos -
        otherwise the position servo would immediately start pulling the joint back toward
        whatever it last held (e.g. still-default home config, if nothing else is driving
        this sim) on the very next physics step, fighting the teleport instead of holding it.
        """
        for name, pos in zip(msg.name, msg.position):
            if name in self.qpos_adr:
                self.data.qpos[self.qpos_adr[name]] = pos
                self.data.qvel[self.dof_adr[name]] = 0.0
                self.target_positions[name] = pos
        mujoco.mj_forward(self.model, self.data)

    def apply_targets(self):
        for joint_name, aid in self.actuator_id.items():
            self.data.ctrl[aid] = self.target_positions[joint_name]

    def step_physics(self, between_substeps=None):
        """Runs steps_per_render physics substeps, calling between_substeps() (if given)
        right before EACH one - see main()'s _step(). tiago_pro_opensot_node.py's control
        loop has no substep concept at all: it publishes one new /opensot/joint_states per
        control_dt (~10ms), one integration step each time. Applying only the latest
        queued command once per OUTER (render) loop iteration - the old behavior here -
        silently coalesces/discards every OpenSoT tick in between whenever this loop's own
        true rate falls below OpenSoT's ~100Hz (which it always does to some degree, see
        the fps warning above): the sim ends up tracking a stepped-down, jumpier version of
        the trajectory OpenSoT actually computed, not what OpenSoT actually intended to
        command. Re-checking for a new command before each substep instead keeps pace with
        OpenSoT's own cadence far more closely, and costs nothing extra when there's
        nothing new to apply."""
        for _ in range(self.steps_per_render):
            if between_substeps is not None:
                between_substeps()
            mujoco.mj_step(self.model, self.data)

    def get_log_entry(self) -> dict:
        """Snapshots the current sim state for offline logging (camera streams excluded).

        Schema matches Dont-Be-Brave/timid's Tiago task exactly (see tasks/tiago.py's
        _split_actions and config/tiago_config.py's data_specs):
          - obs/eef_{side}_pose, obs/joint_pos_opensot, obs/joint_pos_real,
            obs/target_object_pose: all (x,y,z,qx,qy,qz,qw) where poses, ROS quaternion order.
          - actions: single flat (16,) vector - [right_pos(3), right_quat(4), left_pos(3),
            left_quat(4), right_gripper(1), left_gripper(1)] - per _split_actions' slicing.

        Both obs/eef_{side}_pose and the pose columns of actions are the COMMANDED
        /cartesian_interface/{side}/target_pose (see self.commanded_pose / _commanded_pose_cb),
        matching exactly what the real robot pipeline already logs there (it has no
        ground-truth "current achieved pose" signal to use instead, so it echoes the
        commanded value - see _commanded_pose_cb's docstring). This USED to be MuJoCo's own
        achieved ground truth instead, on the theory that sim could afford to be more
        accurate than real and it'd transfer with some tuning - it doesn't: replaying a
        recorded "achieved" pose as a NEW target reintroduces OpenSoT's own tracking lag on
        top of the lag that already happened once during collection, which showed up
        concretely as tiago_replay.py's gripper closing on air (the gripper event fires on
        the original recording's schedule, but the replayed arm hasn't caught up to that
        row's target yet). A policy trained on the achieved-pose signal would hit the same
        lag at inference for the same reason, so this needs to match the real pipeline's
        semantics, not just be more sim-accurate than necessary.

        A side this task never commands (no /cartesian_interface/{side}/target_pose
        publisher running for it) falls back to MuJoCo's own ground truth pos/quat, then to
        zeros if this model has no 'ee_{side}' site either - see the fallback chain below.

        Frame: this MuJoCo scene is fixed-base with base_link coincident with the MuJoCo world
        origin, so raw world-frame poses double as the base_link-frame pose without any extra
        transform (same assumption target_object_pose below, and _commanded_pose_cb, rely on).

        obs/eef_{side}_pos/_rot are MuJoCo's own ground truth in (w,x,y,z) MuJoCo quaternion
        order - sim-only, NOT part of the trained schema (unlike eef_{side}_pose above, these
        are unchanged by the above), harmless/ignored by DataSpec.filter if unused, kept for
        convenience/debugging (e.g. comparing achieved vs. commanded tracking error)."""
        obs = {}
        eef_pose_xyzw = {}
        for side in ('left', 'right'):
            pos, quat_wxyz = self._sim_eef_pos_quatwxyz(side)
            if pos is not None:
                obs[f'eef_{side}_pos'] = pos
                obs[f'eef_{side}_rot'] = quat_wxyz  # (w, x, y, z) - MuJoCo ground truth, sim-only, debug only

            commanded = self.commanded_pose[side]
            if commanded is not None:
                action = commanded
            elif pos is not None:
                action = np.concatenate([pos, quat_wxyz[[1, 2, 3, 0]]])
            else:
                action = np.zeros(7)
            eef_pose_xyzw[side] = action
            obs[f'eef_{side}_pose'] = action

        obs['joint_pos_opensot'] = np.array([self.target_positions[n] for n in self.joint_names])
        obs['joint_pos_real'] = np.array([self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        obs['joint_vel_real'] = np.array([self.data.qvel[self.dof_adr[n]] for n in self.joint_names])

        if self.target_object_qpos_adr is not None:
            adr = self.target_object_qpos_adr
            x, y, z, qw, qx, qy, qz = self.data.qpos[adr:adr + 7]
            obs['target_object_pose'] = np.array([x, y, z, qx, qy, qz, qw])

        # 1.0 = closed, 0.0 = open - matches cartesian_interface_node._gripper_cb's threshold
        # (point.x > 0.5 -> CLOSED) and the inference script's _make_gripper_msg (val >= 0.5
        # -> point.x = 1.0), which together define what a "gripper" action value >=0.5 means
        # for both vive-teleop recording and replay on the real robot.
        right_gripper = 1.0 if self.gripper_status['right'] == 'close' else 0.0
        left_gripper = 1.0 if self.gripper_status['left'] == 'close' else 0.0
        actions = np.concatenate([
            eef_pose_xyzw['right'], eef_pose_xyzw['left'], [right_gripper], [left_gripper],
        ]).astype(np.float32)

        return {'actions': actions, 'obs': obs}

    def _log_step_cb(self):
        self.recorder.record(self.get_log_entry())

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = array.array('d', [self.data.qpos[self.qpos_adr[n]] for n in self.joint_names])
        msg.velocity = array.array('d', [self.data.qvel[self.dof_adr[n]] for n in self.joint_names])
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
    node = MujocoSimNode()

    viewer_ctx = mujoco.viewer.launch_passive(node.model, node.data) if node.use_viewer else None

    # Wall-clock-paced replacement for the old create_timer(episode_log_fps, ...): a ROS
    # timer can only ever fire when spin_once() happens to dispatch it, which made recording
    # inherit both spin_once's one-callback-per-call limit (see MAX_SPINS_PER_STEP above) AND
    # this loop's own true achieved rate - capping it well below episode_log_fps regardless
    # of what fps/episode_log_fps were configured to. See _maybe_log() below for how
    # next_log_time is scheduled (a fixed cadence, not re-anchored to whenever it last
    # fired) and why that distinction turned out to matter a lot in practice.
    log_period = 1.0 / node.episode_log_fps
    next_log_time = time.perf_counter()

    def _maybe_log(now):
        """Fires _log_step_cb() on a FIXED schedule (next_log_time += log_period every
        time it fires), not by re-anchoring to whenever it happened to actually fire
        (next_log_time = now + log_period, the old - buggy - version of this). The
        difference matters a lot in practice: with re-anchoring, a SINGLE iteration that
        runs even a couple ms long (ordinary jitter, not a real stall - see the loop_period
        diagnostic, which stays a rock-solid ~11.13ms average even while this bug was
        active) permanently pushes the next scheduled fire time later too, since the new
        schedule is computed FROM that late timestamp - so the very next (perfectly normal)
        iteration can fall just short of the pushed-back threshold and skip a log
        opportunity it should have gotten. That one skip doesn't get corrected either,
        since the next fire re-anchors again - each bit of ordinary jitter costs a
        permanent recording opportunity instead of a one-off delay. This is what was
        capping recorded fps around ~60Hz despite a verified, steady ~90Hz loop: it's not
        that the loop was actually slow, it's that this gate kept discarding opportunities
        the loop was genuinely providing. A fixed schedule self-corrects instead: an
        iteration running long just means that tick's log call happens a bit late, but the
        NEXT tick is still due at the ideal time, not further delayed by the last one's
        lateness. The clamp below only matters if the loop is persistently (not just for
        one iteration) slower than episode_log_fps - without it, a real, sustained slowdown
        would let next_log_time fall further behind every iteration, then burst-fire many
        queued-up catch-up logs in a row the moment the loop caught back up."""
        nonlocal next_log_time
        if now >= next_log_time:
            node._log_step_cb()
            next_log_time += log_period
            if next_log_time < now - log_period:
                next_log_time = now

    def _substep_update():
        """Runs right before EACH physics substep (see step_physics()'s docstring): pick
        up the newest queued /opensot/joint_states, if one has arrived, and apply it -
        instead of applying only whatever was queued once per whole render iteration."""
        rclpy.spin_once(node, timeout_sec=0)
        node.apply_targets()

    def _step():
        """Returns (physics_dt, spin_dt) - see the timing breakdown below main()'s loop.
        physics_dt now includes the per-substep spin_once calls (see _substep_update) -
        they're not separable from stepping any more, since they're interleaved with it."""
        node.actuate_grippers()
        t0 = time.perf_counter()
        node.step_physics(between_substeps=_substep_update)
        t1 = time.perf_counter()
        # Extra drain for everything NOT on command_topic (gripper toggles, resets,
        # services) - see MAX_SPINS_PER_STEP's comment.
        for _ in range(MAX_SPINS_PER_STEP):
            rclpy.spin_once(node, timeout_sec=0)
        t2 = time.perf_counter()
        return (t1 - t0), (t2 - t1)

    # Accumulators for the periodic timing breakdown (TIMING_LOG_PERIOD_SEC) - see that
    # constant's comment for why this exists. 'total' is compute only (everything in the
    # loop body BEFORE the end-of-iteration sleep - physics/spin/viewer_sync/publish and
    # anything not separately measured, e.g. the get_log_entry() call when it fires).
    # 'loop_period' is the actual wall-clock time from one iteration's start to the next -
    # i.e. compute + sleep - and is the number that actually determines achieved Hz. The
    # two can and do diverge: 'total' can be a small fraction of the 1/fps budget while
    # 'loop_period' still comes out much longer, because time.sleep() on Linux has no hard
    # guarantee of waking up at the requested time - under any real scheduler contention
    # (several ROS nodes/containers sharing a CPU) it commonly overshoots by several ms,
    # and a naive time.sleep(sleep_time) has no way to compensate. 'sleep_requested' vs
    # 'sleep_actual' below makes that overshoot directly visible instead of inferred.
    timing = {
        'physics': 0.0, 'spin': 0.0, 'viewer_sync': 0.0, 'publish': 0.0, 'total': 0.0,
        'sleep_requested': 0.0, 'sleep_actual': 0.0, 'loop_period': 0.0,
    }
    timing_iters = 0
    last_timing_log = time.perf_counter()
    prev_iter_start = None

    def _sleep_precise(duration):
        """Sleeps most of `duration` coarsely, then busy-spins on perf_counter() for the
        last ~1.5ms - see the comment on 'loop_period' above for why a plain time.sleep()
        can't be trusted to hit a short duration precisely. Trades a bit of CPU (up to
        ~1.5ms of busy-waiting per iteration) for actually achieving the requested pacing
        instead of silently overshooting it every single iteration."""
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
                # Everything here (including logging) is inside the lock too: the sim/*
                # services write data.qpos directly, and the passive viewer reads data from
                # its own render thread - an unlocked reset/log read would race it.
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
        node.recorder.save_and_clear(success=False, attempt_index=node.episode_idx)  # outcome unknown at shutdown/Ctrl+C
        if viewer_ctx is not None:
            viewer_ctx.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
