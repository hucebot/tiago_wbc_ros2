"""Unit tests for HomingInterpolator (pure logic, no ROS / OpenSoT)."""

import numpy as np
import pytest
from tiago_control_node.homing import HomingInterpolator

QIDX = {"j0": 0, "j1": 1, "j2": 2}


class Clock:
    """Manually-advanced clock injected as `time_fn`."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def make(**kw):
    clk = Clock()
    return HomingInterpolator(time_fn=clk, **kw), clk


def test_request_activates_but_not_started():
    h, _ = make()
    assert not h.active and not h.started
    h.request({"j1": 1.0})
    assert h.active and not h.started


def test_start_and_linear_interpolation():
    h, clk = make(duration=1.0)
    h.request({"j1": 4.0})
    h.start(np.zeros(3), QIDX)
    assert h.started

    s, qr = h.q_ref()  # t = 0
    assert s == 0.0
    np.testing.assert_allclose(qr, [0.0, 0.0, 0.0])

    clk.t = 0.5
    s, qr = h.q_ref()
    assert s == pytest.approx(0.5)
    np.testing.assert_allclose(qr, [0.0, 2.0, 0.0])

    clk.t = 5.0  # past the end -> clamped
    s, qr = h.q_ref()
    assert s == 1.0
    np.testing.assert_allclose(qr, [0.0, 4.0, 0.0])


def test_rms_error_only_over_targeted_joints():
    h, _ = make()
    h.request({"j0": 0.0, "j2": 4.0})
    h.start(np.zeros(3), QIDX)
    q = np.array([0.0, 99.0, 1.0])  # j1 not a target -> ignored; j2 off by 3
    assert h.rms_error(q, QIDX) == pytest.approx(3.0)


def test_should_finish_needs_interpolation_done():
    h, clk = make(duration=1.0, settle=2.0, tol=0.05)
    h.request({"j0": 1.0})
    h.start(np.zeros(3), QIDX)

    clk.t = 0.5
    assert not h.should_finish(0.5, 0.01)  # s < 1

    clk.t = 1.0
    assert h.should_finish(1.0, 0.01)  # s == 1 and converged

    clk.t = 2.0
    assert not h.should_finish(1.0, 0.5)  # not converged, not timed out

    clk.t = 3.5  # > duration + settle
    assert h.should_finish(1.0, 0.5)  # timed out


def test_finish_clears_state():
    h, _ = make()
    h.request({"j0": 1.0})
    h.start(np.zeros(3), QIDX)
    h.finish()
    assert not h.active and not h.started
