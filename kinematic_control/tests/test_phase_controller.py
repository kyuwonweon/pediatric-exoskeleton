"""LegPipeline tests (no ROS needed). Run from kinematic_control/:  python -m pytest tests/test_phase_controller.py"""
import math

import numpy as np
import pytest

from kinematic_control.estimation.phase_portrait.phase_portrait import load_lookup
from kinematic_control.frames import LegFrame
from kinematic_control.ros.phase_controller_node import DT, LegPipeline
from kinematic_control.torque_map import TorqueMap


@pytest.fixture(scope="module")
def parts():
    return TorqueMap.load(peak_Nm=2.0), load_lookup()


def swing(seconds, period=1.1, lo=-12.0, hi=28.0):
    """Walking-like thigh swing: angle and velocity at 50 Hz."""
    t = np.arange(0, seconds, DT)
    mid, amp = (hi + lo) / 2, (hi - lo) / 2
    w = 2 * np.pi / period
    return t, mid + amp * np.sin(w * t), amp * w * np.cos(w * t)


def test_swing_gives_phase_and_bounded_torque(parts):
    pipe = LegPipeline("right", *parts)
    t, a, v = swing(12)
    rows = np.array([pipe.step(ti, ai, vi) for ti, ai, vi in zip(t, a, v)])
    late = rows[t > 6]                                   # after warm-up and soft start
    assert np.all(late[:, 3] == 1)                       # walking
    assert np.all((late[:, 2] >= 0) & (late[:, 2] < 1))  # phase in range
    assert np.max(np.abs(late[:, 5])) > 0.3              # torque actually comes on
    assert np.max(np.abs(rows[:, 5])) <= 2.0 + 1e-9      # never above the cap
    assert np.all(rows[:, 6] == 0) and np.all(rows[:, 7] == 0)


def test_no_torque_while_warming_up(parts):
    pipe = LegPipeline("left", *parts)
    row = pipe.step(0.0, 5.0, 0.0)
    assert math.isnan(row[2]) and row[5] == 0.0


def test_counts_dropped_ticks(parts):
    pipe = LegPipeline("right", *parts)
    pipe.step(0.00, 0.0, 0.0)
    assert pipe.step(0.02, 0.0, 0.0)[7] == 0
    assert pipe.step(0.10, 0.0, 0.0)[7] == 3             # 0.04, 0.06, 0.08 missing


def test_fault_latches_until_reset(parts):
    pipe = LegPipeline("right", *parts)
    pipe.step(0.00, 0.0, 0.0)
    assert pipe.step(0.02, 40.0, 0.0)[6] == 1            # 40 deg jump in one tick
    assert pipe.step(0.04, 40.0, 0.0)[6] == 1            # stays latched
    pipe.reset()
    assert pipe.step(0.06, 40.0, 0.0)[6] == 0


def test_refuses_wrong_board_sign(parts):
    with pytest.raises(ValueError):
        LegPipeline("left", *parts, frame=LegFrame("left", sign=-1))
