"""Phase portrait tests. Run from kinematic_control/:  python -m pytest tests/test_phase_portrait.py"""
import numpy as np

from kinematic_control.estimation.phase_portrait import PhasePortrait
from kinematic_control.estimation.phase_portrait.phase_portrait import run, signed_diff

DT = 0.02


def sine_walk(seconds=20.0, period=1.1, amp=20.0, offset=5.0):
    """Thigh angle like walking: + = flexion, stride starts at max flexion."""
    t = np.arange(0, seconds, DT)
    return t, offset + amp * np.cos(2 * np.pi * t / period)


def test_smooth_gets_slope_of_a_sine():
    t, angle = sine_walk()
    pp = PhasePortrait()
    out = [pp.smooth(a) for a in angle]
    true_vel = -20 * 2 * np.pi / 1.1 * np.sin(2 * np.pi * t / 1.1)
    vel = np.array([o[1] for o in out])
    assert np.max(np.abs(vel[20:] - true_vel[20:])) < 0.05 * np.max(np.abs(true_vel))


def test_raw_phase_increases_counter_clockwise():
    angles = np.linspace(0, 2 * np.pi, 50, endpoint=False)
    phis = [PhasePortrait.raw_phase(np.cos(a), np.sin(a)) for a in angles]
    assert np.allclose(phis, angles)


def test_hold_never_goes_backward_but_wraps():
    pp = PhasePortrait()
    # 1.4 is a step back (held); 6.25 after 0.05 is a small step back across 0 (held)
    out = [pp.hold(p, False) for p in [1.0, 1.5, 1.4, 1.6, 3.0, 4.5, 6.2, 0.05, 6.25]]
    assert out == [1.0, 1.5, 1.5, 1.6, 3.0, 4.5, 6.2, 0.05, 0.05]


def test_signed_diff_across_zero():
    assert np.isclose(signed_diff(0.1, 2 * np.pi - 0.1), 0.2)
    assert np.isclose(signed_diff(2 * np.pi - 0.1, 0.1), -0.2)


def test_sine_walk_phase_is_linear():
    """On a pure sine both y options give a circle, so phi/2pi should track time exactly."""
    for y_signal in ("integral", "velocity"):
        t, angle = sine_walk()
        phase, _ = run(angle, y_signal=y_signal)
        ok = np.isfinite(phase) & (t > 6)
        true = (t / 1.1) % 1.0
        err = np.abs((phase[ok] - true[ok] + 0.5) % 1 - 0.5)
        assert err.mean() < 0.03, (y_signal, err.mean())


def test_stop_freezes_then_resumes():
    t, angle = sine_walk(seconds=30)
    stop = (t > 12.0) & (t < 16.0)
    angle = angle.copy()
    angle[stop] = angle[np.argmax(stop) - 1]     # leg stands still for 4 s
    angle[t >= 16.0] = 5 + 20 * np.cos(2 * np.pi * (t[t >= 16.0] - 4.0) / 1.1)  # resumes where it stopped
    pp = PhasePortrait()
    phase, walking = [], []
    for a in angle:
        phase.append(pp.step(a))
        walking.append(pp.walking)
    walking = np.array(walking)
    assert not walking[(t > 14.0) & (t < 16.0)].any()   # stopped is detected within 1.5 s
    assert walking[t > 20.0].all()                       # and walking resumes


def test_resume_after_brief_stop_when_slightly_ahead():
    """A false stop: phase held, live phase moves a little ahead -> resume at once, not a cycle later."""
    pp = PhasePortrait(resume_tol=0.3, resume_ahead=0.8)
    pp.last_phi = 1.0
    assert pp.hold(1.0, stopped=True) == 1.0 and pp.held
    assert pp.hold(1.5, stopped=False) == 1.5 and not pp.held     # 0.5 rad ahead: resume
    pp.hold(1.5, stopped=True)
    assert pp.hold(2.6, stopped=False) == 1.5 and pp.held         # 1.1 rad ahead: too far, wait
