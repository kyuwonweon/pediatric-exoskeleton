"""Safety tests. Run from kinematic_control/:  python -m pytest tests/test_safety.py"""
import numpy as np

from kinematic_control.safety import Limits, Safety, make_leg_safety

DT = 0.02


def run(s, n, torque=1.0, walking=True, angle=10.0, velocity=0.0):
    return [s.step(torque, walking, angle, velocity) for _ in range(n)]


def test_off_until_enabled():
    s = Safety()
    assert max(run(s, 100)) == 0.0


def test_soft_start_takes_soft_start_s():
    s = Safety(Limits(soft_start_s=2.0))
    s.enable()
    out = run(s, int(3.0 / DT))
    assert 0.45 < out[int(1.0 / DT) - 1] < 0.55      # half way at 1 s
    assert out[-1] == 1.0                            # full after 2 s


def test_not_walking_ramps_down_and_restarts_soft():
    s = Safety()
    s.enable()
    run(s, 200)                                      # at full torque
    out = run(s, 10, walking=False)
    assert out[0] < 1.0 and out[-1] == 0.0           # down within 0.1 s, not instantly
    back = run(s, 5)
    assert back[-1] < 0.1                            # soft start again, not straight to 1


def test_rate_limit():
    s = Safety(Limits(max_rate=20.0, soft_start_s=0.02))
    s.enable()
    out = run(s, 10, torque=2.0)
    assert np.allclose(np.diff([0.0] + out[:5]), 0.4)   # 20 Nm/s * 0.02 s per tick


def test_cap():
    s = Safety(Limits(max_torque=2.0, soft_start_s=0.02, max_rate=1e6))
    s.enable()
    assert run(s, 3, torque=5.0)[-1] == 2.0
    assert run(s, 3, torque=-5.0)[-1] == -2.0


def test_angle_limit_blocks_only_pushing_further_out():
    def fresh():                                     # right leg: flex 60, ext -15
        s = make_leg_safety("right")
        s.limits.soft_start_s, s.limits.max_rate = 0.02, 1e6
        s.enable()
        return s
    assert run(fresh(), 3, torque=+1.0, angle=65.0)[-1] == 0.0    # more flexion past the limit: no
    assert run(fresh(), 3, torque=-1.0, angle=65.0)[-1] == -1.0   # back toward the middle: yes
    assert run(fresh(), 3, torque=-1.0, angle=-16.0)[-1] == 0.0
    assert run(fresh(), 3, torque=+1.0, angle=-16.0)[-1] == 1.0


def test_leg_limits_are_copies():
    a = make_leg_safety("right")
    a.limits.flex_limit = 10
    assert make_leg_safety("right").limits.flex_limit == 60


def test_fault_cuts_instantly_and_latches():
    s = Safety()
    s.enable()
    run(s, 200)
    assert s.step(1.0, True, 40.0, 0.0) == 0.0       # 30 deg jump in one tick
    assert s.fault is not None and not s.enabled
    assert max(run(s, 50, angle=40.0)) == 0.0        # stays off even with good input
    s.enable()
    assert not s.enabled                             # can't enable while latched
    s.reset()
    assert not s.enabled                             # reset doesn't turn torque back on
    s.enable()
    assert run(s, 200, angle=40.0)[-1] == 1.0


def test_nan_is_a_fault():
    s = Safety()
    s.enable()
    assert s.step(float("nan"), True, 10.0, 0.0) == 0.0 and s.fault == "non-finite input"


def test_stop_fades_without_a_step():
    s = Safety(Limits(fade_s=0.2, max_rate=1000.0))   # rate limit out of the way: test the fade itself
    s.enable()
    run(s, 200)                                      # at full torque (1 Nm)
    out = run(s, 15, walking=False)
    steps = np.abs(np.diff([1.0] + out))
    assert steps.max() <= 0.1 + 1e-9                 # <= 10 % of the torque per tick
    assert out[9] == 0.0                             # gone after fade_s


def test_brief_stop_ramps_back_fast():
    s = Safety(Limits(fade_s=0.2, soft_start_s=2.0))
    s.enable()
    run(s, 200)
    run(s, 2, walking=False)                         # 0.04 s flicker: gain 1 -> 0.8
    back = run(s, 3)
    assert back[-1] == 1.0                           # back at the fade rate, not a 2 s soft start


def test_rate_scales_with_peak():
    from kinematic_control.safety import rate_for_peak
    assert rate_for_peak(0.5) == 5.0                 # 0.1 Nm per 50 Hz tick
    assert rate_for_peak(2.0) == 20.0
    assert rate_for_peak(5.0) == 20.0                # never above the ceiling
    s = make_leg_safety("left", DT, peak_Nm=0.5)
    assert s.limits.max_rate == 5.0
    assert make_leg_safety("left", DT).limits.max_rate == 20.0   # no peak -> unchanged
