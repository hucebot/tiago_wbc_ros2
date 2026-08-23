#!/usr/bin/env python3
"""Buffers one episode's step data and saves-or-discards it as an HDF5 "demo_N" group when
the episode ends. Plain Python - no ROS2, no MuJoCo - so the logging/schema logic can be
read, tested, and changed without touching anything that knows about the simulator or robot.

Used by mujoco_sim_node.py: that node calls record() once per logged step (with a dict it
builds from the live MjData), pause()/resume() around a reset (so transient reset states
never end up in the training data), and save_and_clear() when an episode ends.

Schema matches Dont-Be-Brave/timid's Tiago task exactly (see tasks/tiago.py's
_split_actions and config/tiago_config.py's data_specs):
  - obs/eef_{side}_pose, obs/joint_pos_opensot, obs/joint_pos_real, obs/target_object_pose:
    all (x,y,z,qx,qy,qz,qw), ROS quaternion order.
  - actions: single flat (16,) vector - [right_pos(3), right_quat(4), left_pos(3),
    left_quat(4), right_gripper(1), left_gripper(1)] - per _split_actions' slicing.
Each entry passed to record() must be a {'actions': ..., 'obs': {...}} dict already matching
that schema - this file doesn't build it, just buffers/writes it (see
mujoco_sim_node.py's get_log_entry()).
"""
import os

import numpy as np
import h5py


class EpisodeRecorder:
    def __init__(self, log_path: str, save_failed_episodes: bool, logger=None):
        self.log_path = log_path
        self.save_failed_episodes = save_failed_episodes
        self._logger = logger
        self.log_buffer = []
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

    def save_and_clear(self, success: bool, attempt_index: int):
        """Writes the buffered episode to `log_path` as data/demo_N (unless it failed and
        save_failed_episodes is False, in which case it's discarded) and clears the buffer
        either way. `attempt_index` is just recorded as an attrs for debugging - it's the
        caller's running count of reset attempts, success or not. Returns the saved group's
        path, or None if nothing was saved."""
        if not self.log_buffer:
            return None
        if not success and not self.save_failed_episodes:
            self._log(
                f"Episode {attempt_index} failed ({len(self.log_buffer)} steps) - discarding, "
                "not saved (save_failed_episodes:=true to keep failures too).")
            self.log_buffer = []
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
        self._log(
            f"Saved data/{demo_name} ({len(self.log_buffer)} steps, success={success}) to {self.log_path}")
        self.saved_episode_idx += 1
        self.log_buffer = []
        return f'{self.log_path}::data/{demo_name}'
