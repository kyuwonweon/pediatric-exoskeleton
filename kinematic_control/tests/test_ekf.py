"""EKF tests on fake walking made by the gait model itself, so the true state is known exactly.

Run from kinematic_control/:  python -m tests.test_ekf   (or pytest, if installed)
"""
import numpy as np

from kinematic_control.ekf import PhaseEKF
from kinematic_control.gait_model import MODEL_PATH, GaitModel

DT = 0.02  # 50 Hz, like the exo


def make_ekf(model, start_phase):
    return PhaseEKF(model, DT, x0=[start_phase, 0.9, 1.2, 0.0],
                    P0=np.diag([0.1, 0.05, 0.05, 25.0]),
                    Q=np.diag([1e-6, 1e-5, 1e-6, 1e-4]),
                    R=np.diag([2.0 ** 2, 20.0 ** 2]))


def fake_walk(model, true_state, seconds, seed=0):
    """Yield (true state, noisy sensor reading) every tick for a steady walk."""
    rng = np.random.default_rng(seed)
    x = np.array(true_state, dtype=float)
    reader = make_ekf(model, 0.0)  # only used for its measurement() function
    for _ in range(int(seconds / DT)):
        x[0] = (x[0] + x[1] * DT) % 1
        yield x.copy(), reader.measurement(x) + rng.normal(0, [1.0, 10.0])


def phase_error(a, b):
    """Shortest distance between two phases, so 0.99 vs 0.01 is 0.02, not 0.98."""
    return abs((a - b + 0.5) % 1 - 0.5)


def run(model, start_phase, true_state, seconds=10, warmup=50):
    """Collect the first `warmup` readings (1 s), initialize from them, then run the EKF."""
    ekf = make_ekf(model, start_phase)
    first, errors = [], []
    for truth, z in fake_walk(model, true_state, seconds):
        if len(first) < warmup:
            first.append(z)
            if len(first) == warmup:
                ekf.initialize(first)
            continue
        x = ekf.step(z)
        errors.append(phase_error(x[0], truth[0]))
    return x, np.mean(errors[-100:])  # final state, mean phase error over the last 2 s


def test_converges_from_any_start():
    model = GaitModel.load(MODEL_PATH)
    true_state = [0.0, 0.85, 1.3, 5.0]
    for start_phase in (0.0, 0.25, 0.5, 0.75):
        x, err = run(model, start_phase, true_state)
        assert err < 0.02, f"start {start_phase}: phase error {err:.3f}"
        assert abs(x[1] - 0.85) < 0.05 and abs(x[2] - 1.3) < 0.1 and abs(x[3] - 5.0) < 1.5, x


def test_tracks_other_conditions():
    model = GaitModel.load(MODEL_PATH)
    for true_state in ([0.3, 0.75, 1.0, -8.0], [0.6, 1.0, 1.5, 0.0], [0.1, 0.8, 1.2, 10.0]):
        x, err = run(model, 0.0, true_state)
        assert err < 0.02, f"{true_state}: phase error {err:.3f}"
        assert abs(x[3] - true_state[3]) < 1.5, f"{true_state}: incline {x[3]:.1f}"


def test_states_stay_in_range():
    model = GaitModel.load(MODEL_PATH)
    ekf = make_ekf(model, 0.0)
    rng = np.random.default_rng(1)
    for _ in range(500):  # garbage readings
        x = ekf.step(rng.normal(0, [40.0, 300.0]))
        assert np.all(x >= PhaseEKF.LOWER) and np.all(x <= PhaseEKF.UPPER), x


if __name__ == "__main__":
    for test in (test_converges_from_any_start, test_tracks_other_conditions, test_states_stay_in_range):
        test()
        print(f"passed: {test.__name__}")
