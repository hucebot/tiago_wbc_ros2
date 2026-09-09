"""Time-parametrised joint-space homing interpolation.

`HomingInterpolator` owns only the timing + interpolation + completion logic --
no ROS or OpenSoT dependencies -- so it is unit-testable. The control loop drives
it each tick and does the actual task poking:

    homing.request({"arm_left_1_joint": 0.3, ...})   # from the /opensot/home_cmd cb
    ...
    if homing.active:
        if not homing.started:
            homing.start(q, q_index_map)
        s, q_ref = homing.q_ref()
        # ... push q_ref into q_homing / postural / the arm Cartesian refs ...
        err = homing.rms_error(q, q_index_map)
        if homing.should_finish(s, err):
            homing.finish()
            # ... publish /opensot/home_done ...
"""

import time

import numpy as np


class HomingInterpolator:
    def __init__(
        self,
        duration: float = 0.5,
        settle: float = 2.0,
        tol: float = 0.05,
        time_fn=time.perf_counter,
    ):
        self.duration = duration  # seconds of linear interpolation
        self.settle = settle  # extra seconds allowed to converge afterwards
        self.tol = tol  # rad RMS joint error that counts as "home"
        self._now = time_fn  # injectable for tests

        self.active = False  # a home has been requested and not finished
        self._running = False  # start() has been called for this request
        self._targets: dict = {}  # joint name -> target angle (homed joints only)
        self._start_q = None
        self._target_q = None
        self._t0 = 0.0

    @property
    def started(self) -> bool:
        return self._running

    def request(self, joint_targets: dict) -> None:
        """Queue a home to `joint_targets` (name -> angle). start() runs next tick."""
        self._targets = dict(joint_targets)
        self.active = True
        self._running = False

    def start(self, q, q_index_map: dict) -> None:
        """Latch the start configuration and build the full target vector."""
        self._start_q = np.copy(q)
        self._target_q = np.copy(q)
        for name, angle in self._targets.items():
            i = q_index_map.get(name)
            if i is not None and i < len(self._target_q):
                self._target_q[i] = angle
        self._t0 = self._now()
        self._running = True

    def elapsed(self) -> float:
        return self._now() - self._t0

    def q_ref(self):
        """(s, q_ref) for the current tick. s in [0, 1] is the interpolation phase."""
        s = float(np.clip(self.elapsed() / self.duration, 0.0, 1.0))
        return s, self._start_q + s * (self._target_q - self._start_q)

    def rms_error(self, q, q_index_map: dict) -> float:
        err = 0.0
        for name, angle in self._targets.items():
            i = q_index_map.get(name)
            if i is not None and i < len(q):
                err += (q[i] - angle) ** 2
        return float(err**0.5)

    def timed_out(self) -> bool:
        return self.elapsed() > (self.duration + self.settle)

    def should_finish(self, s: float, err: float) -> bool:
        return s >= 1.0 and (err < self.tol or self.timed_out())

    def finish(self) -> None:
        self.active = False
        self._running = False
