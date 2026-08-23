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

The tiago-pro-mujoco XML is fixed-base and headless (no wheel or head joints), so
base and head commands from OpenSoT are simply not applied here.

Recording runs on its own timer at episode_log_fps, independent of the physics/render
loop's fps - see main()'s loop and _log_step_cb below. episode_log_fps must not exceed
fps, since ROS timers here can only fire as often as the main loop calls spin_once (once
per physics/render iteration); __init__ warns loudly if that's misconfigured.
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

        self.recorder = EpisodeRecorder(episode_log_path, save_failed_episodes, logger=self.get_logger())

        self._reset_robot_home()

        self.command_sub = self.create_subscription(
            JointState, command_topic, self._command_cb, 10)
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

        self.create_timer(1.0 / self.episode_log_fps, self._log_step_cb)

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

    def _sim_eef_pos_quatwxyz(self, side):
        """MuJoCo's own live ground-truth end-effector pose - position and (w,x,y,z)
        quaternion, or (None, None) if this model has no 'ee_{side}' site."""
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

    def apply_targets(self):
        for joint_name, aid in self.actuator_id.items():
            self.data.ctrl[aid] = self.target_positions[joint_name]

    def step_physics(self):
        for _ in range(self.steps_per_render):
            mujoco.mj_step(self.model, self.data)

    def get_log_entry(self) -> dict:
        """Snapshots the current sim state for offline logging (camera streams excluded).

        Schema matches Dont-Be-Brave/timid's Tiago task exactly (see tasks/tiago.py's
        _split_actions and config/tiago_config.py's data_specs):
          - obs/eef_{side}_pose, obs/joint_pos_opensot, obs/joint_pos_real,
            obs/target_object_pose: all (x,y,z,qx,qy,qz,qw) where poses, ROS quaternion order.
          - actions: single flat (16,) vector - [right_pos(3), right_quat(4), left_pos(3),
            left_quat(4), right_gripper(1), left_gripper(1)] - per _split_actions' slicing.

        Both obs/eef_{side}_pose and the pose columns of actions are the ACTUAL end-effector
        pose (MuJoCo's own ground truth, i.e. what obs/eef_{side}_pos/_rot below duplicate in
        a different quaternion order), not the commanded /cartesian_interface/{side}/target_pose
        - deliberately, even for a side this task never commands: the recorded state should be
        what the effector is really doing (including any incidental whole-body coupling from
        the torso/base helping the OTHER side reach its target) rather than a value the arm was
        merely asked for and may not have exactly achieved.

        NOTE this is a deliberate sim-vs-real divergence, not a match: on the real robot,
        eef_{side}_pose is the commanded Vive/teleop pose echoed through
        /cartesian_interface/{side}/target_pose (there's no ground-truth "current achieved
        pose" signal available in that pipeline to use instead). Sim has no such limitation -
        MuJoCo's ground truth is free to read - so this uses it anyway: the sim is meant to be
        a testbed for what representation/tuning actually works for the task, on the
        assumption that whatever works here should transfer to the real robot with some
        additional tuning, not to byte-for-byte reproduce the real pipeline's own workarounds
        for signals it happens not to have. Revisit this if sim-trained policies don't
        transfer cleanly and the eef_pose semantics turn out to be why.

        Frame: this MuJoCo scene is fixed-base with base_link coincident with the MuJoCo world
        origin, so raw world-frame site_xpos/site_xmat doubles as the base_link-frame pose
        without any extra transform (same assumption target_object_pose below relies on).

        obs/eef_{side}_pos/_rot duplicate the same ground truth in (w,x,y,z) MuJoCo quaternion
        order - NOT part of the trained schema, harmless/ignored by DataSpec.filter if unused,
        kept for convenience/debugging."""
        obs = {}
        eef_pose_xyzw = {}
        for side in ('left', 'right'):
            pos, quat_wxyz = self._sim_eef_pos_quatwxyz(side)
            action = np.zeros(7)
            if pos is not None:
                obs[f'eef_{side}_pos'] = pos
                obs[f'eef_{side}_rot'] = quat_wxyz  # (w, x, y, z) - MuJoCo ground truth, sim-only
                action = np.concatenate([pos, quat_wxyz[[1, 2, 3, 0]]])
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

    try:
        while rclpy.ok() and (viewer_ctx is None or viewer_ctx.is_running()):
            start = time.perf_counter()

            if viewer_ctx is not None:
                # spin_once is inside the lock too: the sim/* services write data.qpos
                # directly, and the passive viewer reads data from its own render thread -
                # an unlocked reset would race it.
                with viewer_ctx.lock():
                    node.apply_targets()
                    node.actuate_grippers()
                    node.step_physics()
                    rclpy.spin_once(node, timeout_sec=0)
                viewer_ctx.sync()
            else:
                node.apply_targets()
                node.actuate_grippers()
                node.step_physics()
                rclpy.spin_once(node, timeout_sec=0)

            node.publish_joint_state()
            node.publish_target_object_pose()
            node.publish_settled_at_home()

            elapsed = time.perf_counter() - start
            sleep_time = (1.0 / node.fps) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
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
