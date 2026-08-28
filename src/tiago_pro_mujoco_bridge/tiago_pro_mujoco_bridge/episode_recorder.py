#!/usr/bin/env python3
"""Buffers one episode's step data and saves-or-discards it as an HDF5 "demo_N" group when
the episode ends. Plain Python - no ROS2, no MuJoCo - so the logging/schema logic can be
read, tested, and changed without touching anything that knows about the simulator or robot.

Used by mujoco_sim_node.py: that node calls record() once per logged step (with a dict it
builds from the live MjData), pause()/resume() around a reset (so transient reset states
never end up in the training data), and save_and_clear() when an episode ends. save_and_clear()
also pauses as part of clearing the buffer, not just reset - see its docstring for why that
matters (episode_orchestrator_node.py saves before it resets, and the recording timer is
independent of that call sequence).

Schema matches Dont-Be-Brave/timid's Tiago task exactly (see tasks/tiago.py's
_split_actions and config/tiago_config.py's data_specs):
  - obs/eef_{side}_pose, obs/joint_pos_opensot, obs/joint_pos_real, obs/target_object_pose:
    all (x,y,z,qx,qy,qz,qw), ROS quaternion order.
  - actions: single flat (16,) vector - [right_pos(3), right_quat(4), left_pos(3),
    left_quat(4), right_gripper(1), left_gripper(1)] - per _split_actions' slicing.
Each entry passed to record() must be a {'actions': ..., 'obs': {...}} dict already matching
that schema - this file doesn't build it, just buffers/writes it (see
mujoco_sim_node.py's get_log_entry()).

Each saved demo_N group gets 'fps' (MEASURED from time.monotonic() between the first and
last recorded step - NOT time.time(): that's the wall clock, which NTP/chrony can slew or
step at any moment for reasons that have nothing to do with how fast this loop is actually
running, and doing exactly that once silently manufactured a bunch of fake "slow step" gaps
here before this was switched to a monotonic clock) and 'requested_fps' (the log_fps passed
to __init__, i.e. mujoco_sim_node.py's episode_log_fps parameter) attrs. These can differ:
mujoco_sim_node.py's main loop paces recording to episode_log_fps itself (a plain wall-clock
accumulator, not a ROS timer - a ROS timer used to be the mechanism here, but that let a
busy ~100Hz subscription starve it out regardless of configuration, see that file's main());
if physics stepping + ROS overhead take longer per iteration than 1/fps allows, the real
recording rate silently falls below whatever was configured, with nothing before this
warning that it happened. 'fps' is what a replay tool or training should actually trust;
'requested_fps' is kept only for comparing against it.
"""
import os
import time

import numpy as np
import h5py


class EpisodeRecorder:
    def __init__(self, log_path: str, save_failed_episodes: bool, log_fps: float, logger=None):
        self.log_path = log_path
        self.save_failed_episodes = save_failed_episodes
        self.requested_fps = log_fps
        self._logger = logger
        self.log_buffer = []
        self._timestamps = []  # time.monotonic() at each record() call, for the MEASURED fps -
                                # NOT time.time(), see module docstring's 'fps' attr paragraph
        self.paused = False
        # counts only episodes actually written to disk - keeps saved filenames contiguous
        # even when failures are skipped. log_path is opened in append mode and persists
        # across process restarts (crash, code update, ...) - starting this at 0 unconditionally
        # would collide with demo_N groups an earlier run already wrote into the same file
        # (h5py raises on create_group of a name that already exists), so resume from
        # whatever's actually in the file instead.
        self.saved_episode_idx = self._next_demo_idx()

    def _next_demo_idx(self) -> int:
        if not os.path.exists(self.log_path):
            return 0
        try:
            with h5py.File(self.log_path, 'r') as f:
                data_grp = f.get('data')
                if data_grp is None or len(data_grp) == 0:
                    return 0
                indices = [int(name.split('_', 1)[1]) for name in data_grp.keys()
                           if name.startswith('demo_') and name.split('_', 1)[1].isdigit()]
                return max(indices) + 1 if indices else 0
        except OSError:
            return 0

    def _log(self, msg):
        if self._logger is not None:
            self._logger.info(msg)

    def pause(self):
        """Stop appending steps - e.g. while a reset is in flight and the sim is passing
        through states (robot mid-resync, object mid-respawn) that shouldn't appear as
        training data."""
        self.paused = True

    def resume(self):
        self.paused = False

    def record(self, entry: dict):
        if not self.paused:
            self.log_buffer.append(entry)
            self._timestamps.append(time.monotonic())

    def save_and_clear(self, success: bool, attempt_index: int):
        """Writes the buffered episode to `log_path` as data/demo_N (unless it failed and
        save_failed_episodes is False, in which case it's discarded) and clears the buffer
        either way. `attempt_index` is just recorded as an attrs for debugging - it's the
        caller's running count of reset attempts, success or not. Returns the saved group's
        path, or None if nothing was saved.

        Also pauses recording, same as pause() - episode_orchestrator_node.py calls this
        (via /mujoco_bridge/sim/save_episode_log) BEFORE it calls reset_robot_home (which is
        what used to be the only place pause() got called). mujoco_sim_node.py's _log_step_cb
        timer is independent of that call sequence, so without pausing here too, the gap
        between "buffer cleared" and "reset_robot_home actually runs" left recording live
        with an empty buffer - long enough for a stray frame or two of the just-finished
        episode's final held pose to get appended, then carried (pause alone doesn't clear
        what's already buffered) all the way through to become the START of the NEXT saved
        episode once resume() reopens it. Pausing at the same point the buffer is cleared
        closes that gap instead of relying on callers to pause immediately afterward."""
        self.paused = True
        if not self.log_buffer:
            return None
        if not success and not self.save_failed_episodes:
            self._log(
                f"Episode {attempt_index} failed ({len(self.log_buffer)} steps) - discarding, "
                "not saved (save_failed_episodes:=true to keep failures too).")
            self.log_buffer = []
            self._timestamps = []
            return None

        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        demo_name = f'demo_{self.saved_episode_idx}'
        # Append mode: one persistent file across the whole collection run, one new
        # top-level group per episode. Opened/closed per episode (not held open for the
        # run's duration) so a crash mid-run can't corrupt already-saved demos.
        with h5py.File(self.log_path, 'a') as f:
            # Dont-Be-Brave's Tiago task reads h5py.File(fpath, "r")["data"][demo_name] -
            # the top-level "data" group is required, not optional.
            data_grp = f.require_group('data')
            grp = data_grp.create_group(demo_name)
            grp.create_dataset('actions', data=np.stack([e['actions'] for e in self.log_buffer]))
            obs_grp = grp.create_group('obs')
            for k in self.log_buffer[0]['obs'].keys():
                obs_grp.create_dataset(k, data=np.stack([e['obs'][k] for e in self.log_buffer]))
            grp.attrs['success'] = bool(success)
            grp.attrs['num_steps'] = len(self.log_buffer)
            grp.attrs['attempt_index'] = attempt_index
            grp.attrs['requested_fps'] = self.requested_fps
            # MEASURED from time.monotonic() between the first and last record() call - NOT
            # requested_fps, which is only what mujoco_sim_node.py's main loop tried to
            # sustain. A replay tool or training should trust this one: see module
            # docstring for why they can legitimately differ (a real loop that can't keep
            # up silently records slower than requested_fps says, with nothing otherwise
            # flagging that it happened). Falls back to requested_fps only in the
            # degenerate single-step case, where there's no elapsed time to measure from.
            span = self._timestamps[-1] - self._timestamps[0]
            measured_fps = (len(self._timestamps) - 1) / span if span > 0 else self.requested_fps
            grp.attrs['fps'] = measured_fps
            if abs(measured_fps - self.requested_fps) > 0.1 * self.requested_fps:
                self._log(
                    f"WARNING: {demo_name} requested {self.requested_fps}Hz but measured "
                    f"{measured_fps:.2f}Hz (>10% off) - the main loop couldn't keep up with "
                    f"the requested rate. Training/replay will use the measured rate.")
                # A whole-episode average can come out low either because the loop was
                # uniformly slow throughout, or because a couple of large one-off stalls
                # (a TF lookup, a service call, a reset, GC, ...) dragged the average down
                # while the steady-state rate elsewhere was fine - these look identical in
                # the single 'measured_fps' number above but need different fixes, so
                # report which one this actually was instead of leaving it ambiguous.
                gaps = np.diff(self._timestamps)
                expected_gap = 1.0 / self.requested_fps
                big = [(i, g) for i, g in enumerate(gaps) if g > 2 * expected_gap]
                if big:
                    worst_idx, worst_gap = max(big, key=lambda ig: ig[1])
                    self._log(
                        f"{demo_name}: {len(big)}/{len(gaps)} step(s) had a gap > 2x the "
                        f"expected {1000 * expected_gap:.2f}ms - worst was {1000 * worst_gap:.1f}ms "
                        f"at step {worst_idx}. A handful of stalls like this, not a uniformly "
                        "slow loop, would explain a low whole-episode average despite a steady "
                        "measured rate elsewhere - check what runs around that step index.")
                else:
                    self._log(
                        f"{demo_name}: no individual gap exceeded 2x the expected "
                        f"{1000 * expected_gap:.2f}ms - the shortfall looks like a uniformly "
                        "slower rate throughout this episode, not a few isolated stalls.")
        self._log(
            f"Saved data/{demo_name} ({len(self.log_buffer)} steps, success={success}, "
            f"measured {measured_fps:.2f}Hz) to {self.log_path}")
        self.saved_episode_idx += 1
        self.log_buffer = []
        self._timestamps = []
        return f'{self.log_path}::data/{demo_name}'
