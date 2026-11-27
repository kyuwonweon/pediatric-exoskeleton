"""Step 4: run the EKF on real dataset people it never saw, as if they were walking with the exo.

The dataset stores each stride squeezed to 150 points. To test the EKF we undo that:
stretch every stride back to its real duration at 50 Hz and glue them end to end, so we get a
continuous thigh angle like the exo sensor would give, with the true phase, stride and incline known.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kinematic_control.data_loader import PHASE, Strides
from kinematic_control.estimation.ekf import PhaseEKF
from kinematic_control.gait_model import GaitModel

DT = 0.02  # 50 Hz, like the exo


@dataclass
class Walk:
    """A continuous walk at 50 Hz with its answer key."""
    angle: np.ndarray          # (T,) thigh angle the sensor would read (deg)
    velocity: np.ndarray       # (T,) thigh velocity (deg/s)
    phase: np.ndarray          # (T,) true phase
    phase_rate: np.ndarray     # (T,) true strides per second
    stride_length: np.ndarray  # (T,) true stride length (m)
    incline: np.ndarray        # (T,) true incline (deg)


def make_walk(strides: Strides, noise_deg: float = 0.5, seed: int = 0) -> Walk:
    """Stretch each stride back to its real duration at 50 Hz and glue them in order.
    Pass strides of ONE person, ONE leg, in time order (one condition, or several to test changes)."""
    parts = {name: [] for name in ("angle", "phase", "phase_rate", "stride_length", "incline")}
    for k in range(len(strides.speed)):
        n = int(round(strides.stride_time[k] / DT))         # e.g. 1.10 s -> 55 samples
        phase = np.arange(n) / n                             # 0, 1/55, ..., 54/55
        parts["angle"].append(np.interp(phase, PHASE, strides.thigh[k]))   # read the curve at those phases
        parts["phase"].append(phase)
        parts["phase_rate"].append(np.full(n, 1 / strides.stride_time[k]))
        parts["stride_length"].append(np.full(n, strides.stride_length[k]))
        parts["incline"].append(np.full(n, strides.incline[k]))
    walk = {name: np.concatenate(p) for name, p in parts.items()}

    # Sensor: angle with a little noise; velocity from the noisy angle, like an encoder.
    rng = np.random.default_rng(seed)
    walk["angle"] = walk["angle"] + rng.normal(0, noise_deg, len(walk["angle"]))
    walk["velocity"] = np.gradient(walk["angle"], DT)
    return Walk(**walk)


def run_ekf(model: GaitModel, walk: Walk, Q, R, warmup: int = 50) -> np.ndarray:
    """Initialize from the first `warmup` ticks, then run the EKF tick by tick. Returns (T, 4) estimates."""
    ekf = PhaseEKF(model, DT, x0=[0.0, 0.9, 1.2, 0.0], P0=np.diag([0.1, 0.05, 0.05, 25.0]), Q=Q, R=R)
    readings = np.stack([walk.angle, walk.velocity], axis=1)
    ekf.initialize(readings[:warmup])
    estimates = np.full((len(readings), 4), np.nan)  # nothing during warm-up
    for t in range(warmup, len(readings)):
        estimates[t] = ekf.step(readings[t])
    return estimates


def phase_error(a, b):
    """Shortest distance between phases, so 0.99 vs 0.01 is 0.02, not 0.98."""
    return np.abs((a - b + 0.5) % 1 - 0.5)


def score(walk: Walk, estimates: np.ndarray, skip_s: float = 3.0) -> dict:
    """Errors after the first few seconds (lock-on time): phase (% of stride), stride (m), incline (deg)."""
    keep = slice(int(skip_s / DT), None)
    est = estimates[keep]
    return {"phase_%": 100 * np.mean(phase_error(est[:, 0], walk.phase[keep])),
            "stride_m": np.mean(np.abs(est[:, 2] - walk.stride_length[keep])),
            "incline_deg": np.mean(np.abs(est[:, 3] - walk.incline[keep]))}


CHANGING = [(1.0, 0), (1.0, 5), (1.0, 10), (1.2, 0), (0.8, -5), (1.0, -10)]


def evaluate_all(strides: Strides, Q, R, verbose: bool = False) -> dict:
    """Every person, gait model fit WITHOUT that person, EKF run on them, leg 1:
    'steady' = each speed/incline as its own walk; 'changing' = one walk through CHANGING back to back,
    which also tests how fast the EKF follows a change. Returns mean errors for both."""
    steady, changing = [], []
    for subject in np.unique(strides.subject):
        model = GaitModel().fit(strides.select(strides.subject != subject))
        for speed in np.unique(strides.speed):
            for incline in np.unique(strides.incline):
                m = (strides.subject == subject) & (strides.speed == speed) & (strides.incline == incline) & (strides.leg == 1)
                if m.sum() < 8:
                    continue
                walk = make_walk(strides.select(m))
                steady.append(score(walk, run_ekf(model, walk, Q, R)))
        walk = changing_walk(strides, subject, CHANGING)
        changing.append(score(walk, run_ekf(model, walk, Q, R)))
        if verbose:
            c = changing[-1]
            print(f"  {subject}: changing walk phase {c['phase_%']:4.1f}%  stride {c['stride_m']:.2f} m  "
                  f"incline {c['incline_deg']:4.1f} deg")

    def mean(results):
        return {k: round(float(np.mean([r[k] for r in results])), 2) for k in results[0]}
    return {"steady": mean(steady), "changing": mean(changing)}


def plot_walk(model: GaitModel, walk: Walk, estimates: np.ndarray, title: str, path) -> None:
    """Truth vs EKF estimate over time: angle, phase, phase error, stride length, incline."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.arange(len(walk.angle)) * DT
    ok = ~np.isnan(estimates[:, 0])
    expected = np.full(len(t), np.nan)
    expected[ok] = model.predict(estimates[ok, 0], estimates[ok, 2], estimates[ok, 3])

    fig, ax = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    ax[0].plot(t, walk.angle, color="0.6", lw=1, label="sensor")
    ax[0].plot(t, expected, color="C0", lw=1, label="EKF expects")
    ax[0].set_ylabel("thigh angle (deg)")
    ax[1].plot(t, walk.phase, color="0.6", lw=1.5, label="true")
    ax[1].plot(t, estimates[:, 0], color="C0", lw=1, label="EKF")
    ax[1].set_ylabel("phase")
    ax[2].plot(t, 100 * phase_error(estimates[:, 0], walk.phase), color="C3", lw=1)
    ax[2].axhline(5, color="0.5", ls="--", lw=1)
    ax[2].set_ylabel("phase error (%)")
    ax[3].plot(t, walk.stride_length, color="0.6", lw=2, label="true")
    ax[3].plot(t, estimates[:, 2], color="C0", lw=1, label="EKF")
    ax[3].set_ylabel("stride length (m)")
    ax[4].plot(t, walk.incline, color="0.6", lw=2, label="true")
    ax[4].plot(t, estimates[:, 3], color="C0", lw=1, label="EKF")
    ax[4].set_ylabel("incline (deg)")
    ax[4].set_xlabel("time (s)")
    for a in ax:
        a.grid(alpha=0.3)
    for a in (ax[0], ax[1], ax[3], ax[4]):
        a.legend(loc="upper right", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def changing_walk(strides: Strides, subject: str, conditions, strides_each: int = 10) -> Walk:
    """One person walking through several (speed, incline) conditions back to back, leg 1."""
    pieces = []
    for speed, incline in conditions:
        m = np.flatnonzero((strides.subject == subject) & (strides.speed == speed)
                           & (strides.incline == incline) & (strides.leg == 1))[:strides_each]
        pieces.append(m)
    return make_walk(strides.select(np.concatenate(pieces)))


Q_DEFAULT = np.diag([1e-6, 1e-5, 1e-5, 1e-2])  # picked by the Q sweep (changing + steady walks)
R_DEFAULT = np.diag([4.5 ** 2, 30.0 ** 2])  # = model-vs-new-person error (4.4 deg, 28 deg/s), measured leave-one-out
PLOT_DIR = Path(__file__).parent.parent / "plots"

if __name__ == "__main__":
    import sys

    from kinematic_control.data_loader import load_walking

    strides = load_walking()
    if "--plot" in sys.argv:
        subject = sys.argv[sys.argv.index("--plot") + 1] if len(sys.argv) > sys.argv.index("--plot") + 1 else "AB05"
        model = GaitModel().fit(strides.select(strides.subject != subject))  # model never saw this person
        conditions = [(1.0, 0), (1.0, 5), (1.0, 10), (1.2, 0), (0.8, -5), (1.0, -10)]
        walk = changing_walk(strides, subject, conditions)
        estimates = run_ekf(model, walk, Q_DEFAULT, R_DEFAULT)
        PLOT_DIR.mkdir(exist_ok=True)
        path = PLOT_DIR / f"ekf_{subject}_changing.png"
        names = ", ".join(f"{v} m/s {i:+d} deg" for v, i in conditions)
        plot_walk(model, walk, estimates, f"{subject} (unseen): {names}", path)
        print(f"saved {path}; errors:", {k: round(v, 2) for k, v in score(walk, estimates).items()})
    else:
        print("EKF on unseen people (mean error after 3 s lock-on):")
        print("  ALL:", {k: round(v, 2) for k, v in evaluate_all(strides, Q_DEFAULT, R_DEFAULT, verbose=True).items()})
