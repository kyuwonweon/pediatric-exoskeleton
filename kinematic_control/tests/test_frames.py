"""Frame conversion tests. Run from kinematic_control/:  python -m pytest tests/test_frames.py"""
import numpy as np
import pytest

from kinematic_control.frames import LegFrame, load_frames, measure_sign, measure_zero, save_frames


def test_standing_is_zero_and_flexion_is_positive_on_both_legs():
    # Like the bench: left angle goes up with flexion, right goes down, frames ~35 deg apart.
    left = LegFrame("left", zero=80.0, sign=+1)
    right = LegFrame("right", zero=45.0, sign=-1)
    assert left.hip_angle(80.0) == 0 and right.hip_angle(45.0) == 0
    assert left.hip_angle(110.0) == 30      # thigh 30 deg forward
    assert right.hip_angle(15.0) == 30      # same physical pose on the other leg
    assert right.hip_velocity(-100.0) == 100


def test_wraps_across_180():
    f = LegFrame("left", zero=170.0, sign=+1)
    assert f.hip_angle(-170.0) == pytest.approx(20.0)   # 170 -> 190 is +20, not -340


def test_measure_zero_median_and_refuses_movement():
    rng = np.random.default_rng(0)
    still = 62.0 + rng.normal(0, 0.1, 250)
    assert measure_zero(still) == pytest.approx(62.0, abs=0.05)
    with pytest.raises(ValueError):
        measure_zero(np.linspace(60, 70, 250))          # leg moved


def test_measure_sign():
    assert measure_sign(np.linspace(50, 90, 100)) == +1
    assert measure_sign(np.linspace(50, 10, 100)) == -1
    with pytest.raises(ValueError):
        measure_sign(np.linspace(50, 53, 100))           # barely moved


def test_save_load(tmp_path):
    frames = {"left": LegFrame("left", 80.0, +1), "right": LegFrame("right", 45.0, -1)}
    save_frames(frames, tmp_path / "frames.json")
    assert load_frames(tmp_path / "frames.json") == frames
