"""Torque map tests: it must never ask for more than peak_Nm, even for odd EKF estimates."""
import numpy as np

from kinematic_control.torque_map import TORQUE_MODEL_PATH, TorqueMap


def test_never_exceeds_peak():
    tmap = TorqueMap.load(TORQUE_MODEL_PATH, peak_Nm=2.0)
    rng = np.random.default_rng(0)
    for _ in range(2000):  # includes strides and inclines far outside the dataset; they get clipped
        tau = tmap.command(rng.uniform(0, 1), rng.uniform(0.3, 1.6), rng.uniform(0.3, 2.5), rng.uniform(-30, 30))
        assert abs(tau) <= 2.0 + 1e-9, tau


def test_shape_level_walking():
    """Level walking: extension (-) right after heel strike, flexion (+) peak before toe-off (~50%)."""
    tmap = TorqueMap.load(TORQUE_MODEL_PATH, peak_Nm=2.0)
    phase = np.linspace(0, 1, 100, endpoint=False)
    tau = np.array([tmap.command(p, 0.9, 1.15, 0.0) for p in phase])
    assert tau[5] < 0
    assert 0.4 < phase[np.argmax(tau)] < 0.65


def test_lead_looks_ahead():
    """lead_s = 0.1 s at 1 stride/s should equal reading the map 0.1 of a stride later."""
    ahead = TorqueMap.load(TORQUE_MODEL_PATH, lead_s=0.1)
    plain = TorqueMap.load(TORQUE_MODEL_PATH)
    assert np.isclose(ahead.command(0.3, 1.0, 1.15, 0.0), plain.command(0.4, 1.0, 1.15, 0.0))
