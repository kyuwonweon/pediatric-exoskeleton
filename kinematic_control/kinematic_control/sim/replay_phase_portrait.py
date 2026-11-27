"""Replay sim: does the phase portrait give the right gait phase, and the right torque?

One held-out person (the phase lookup is fit WITHOUT them) walks continuously:
    1.0 m/s -> 1.2 -> 0.8 -> stop (stand upright 3 s) -> 1.0
Every 20 ms, like the exo:
    thigh angle -> PhasePortrait -> TorqueMap -> Safety -> commanded torque
and, as the answer key, the torque map at the TRUE phase = ideal torque.

Both torques use the same fixed stride length and incline 0, so any difference comes from
phase (and safety), not from stride/incline guesses.
Safety gets the thigh angle as a stand-in for the hip angle (the dataset walk has no separate
hip angle here); walking stays well inside the limits either way.

Run from kinematic_control/:
    python -m kinematic_control.sim.replay_phase_portrait            # one person, 4 figures
    python -m kinematic_control.sim.replay_phase_portrait --all      # summary over all 10
"""
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from kinematic_control.data_loader import Strides, load_walking
from kinematic_control.estimation.phase_portrait import PhasePortrait
from kinematic_control.estimation.phase_portrait.phase_portrait import fit_phase_lookup
from kinematic_control.replay import DT, make_walk, phase_error
from kinematic_control.safety import Safety
from kinematic_control.torque_map import TorqueMap

PLOT_DIR = Path(__file__).resolve().parents[2] / "plots" / "phase_portrait"
STRIDE_M = 1.25        # fixed stride length for the torque map (m), a typical value
PEAK_NM = 2.0          # torque map size
SEGMENTS = [(1.0, 10), (1.2, 10), (0.8, 10), ("stop", 3.0), (1.0, 12)]   # (speed, strides) or ("stop", s)


@dataclass
class SimWalk:
    angle: np.ndarray       # thigh angle (deg), what the sensor reads
    phase: np.ndarray       # true phase, NaN while stopped
    speed: np.ndarray       # m/s, 0 while stopped
    stopped: np.ndarray     # True during the stop (incl. sitting down/standing up ramps)


def build_walk(strides: Strides, subject: str, segments=SEGMENTS, ramp_s: float = 0.5,
               seed: int = 0) -> SimWalk:
    """Glue the person's strides (leg 1, flat) into one walk, with a stop where the thigh moves to
    upright (0 deg) over ramp_s, stands still, and moves back to where the next stride starts."""
    parts = {k: [] for k in ("angle", "phase", "speed", "stopped")}
    used = {}

    def add(angle, phase, speed, stopped):
        parts["angle"].append(angle)
        parts["phase"].append(phase)
        parts["speed"].append(np.full(len(angle), speed))
        parts["stopped"].append(np.full(len(angle), stopped))

    for i, seg in enumerate(segments):
        if seg[0] == "stop":
            nxt = segments[i + 1][0]
            first_next = _strides(strides, subject, nxt, used, peek=True)
            start_next = make_walk(first_next).angle[0]
            last = parts["angle"][-1][-1]
            n_ramp, n_hold = int(ramp_s / DT), int(seg[1] / DT)
            angle = np.concatenate([_smooth_ramp(last, 0.0, n_ramp), np.zeros(n_hold),
                                    _smooth_ramp(0.0, start_next, n_ramp)])
            add(angle, np.full(len(angle), np.nan), 0.0, True)
        else:
            speed, n = seg
            w = make_walk(_strides(strides, subject, speed, used, n=n), noise_deg=0.0)
            add(w.angle, w.phase, speed, False)

    walk = {k: np.concatenate(v) for k, v in parts.items()}
    walk["angle"] = walk["angle"] + np.random.default_rng(seed).normal(0, 0.5, len(walk["angle"]))
    return SimWalk(**walk)


def _strides(strides, subject, speed, used, n=1, peek=False):
    """The next n unused strides of this person at this speed (flat, leg 1), in order."""
    idx = np.flatnonzero((strides.subject == subject) & (strides.speed == speed)
                         & (strides.incline == 0) & (strides.leg == 1))
    start = used.get(speed, 0)
    pick = idx[start:start + n]
    if len(pick) < n:
        raise ValueError(f"{subject} has only {len(idx)} strides at {speed} m/s")
    if not peek:
        used[speed] = start + n
    return strides.select(pick)


def _smooth_ramp(a, b, n):
    s = (1 - np.cos(np.linspace(0, np.pi, n))) / 2
    return a + (b - a) * s


# ---- Run the chain ------------------------------------------------------------------------------
def simulate(walk: SimWalk, lookup, tmap: TorqueMap) -> dict:
    """Every tick: angle -> phase portrait -> torque map -> safety. Logs everything."""
    n = len(walk.angle)
    velocity = np.gradient(walk.angle, DT)
    pp = PhasePortrait(lookup=lookup)
    safety = Safety(dt=DT)
    safety.enable()
    log = {k: np.full(n, np.nan) for k in ("phase", "x", "y", "theta", "theta_dot", "desired",
                                          "command", "ideal", "gain")}
    log["walking"] = np.zeros(n, bool)
    for k in range(n):
        ph = pp.step(walk.angle[k])
        log["theta"][k], log["theta_dot"][k] = pp.angles[-1], _last_velocity(pp)
        if ph is not None:
            log["phase"][k], log["x"][k], log["y"][k] = ph, pp.x, pp.y
        log["walking"][k] = pp.walking
        desired = tmap.command(ph, 0.0, STRIDE_M, 0.0) if ph is not None else 0.0
        log["desired"][k] = desired
        log["command"][k] = safety.step(desired, pp.walking, walk.angle[k], velocity[k])
        log["gain"][k] = safety.gain
        log["ideal"][k] = (tmap.command(walk.phase[k], 0.0, STRIDE_M, 0.0)
                           if np.isfinite(walk.phase[k]) else 0.0)
    return log


def _last_velocity(pp: PhasePortrait) -> float:
    if len(pp.angles) < pp.angles.maxlen:
        return 0.0
    return float(pp.weights[1] @ np.asarray(pp.angles))


# ---- Numbers ------------------------------------------------------------------------------------
def metrics(walk: SimWalk, log: dict) -> dict:
    """phase error while the estimator says walking; torque errors where assist is fully on."""
    walking_truth = np.isfinite(walk.phase)
    est = log["walking"] & walking_truth
    full = est & (log["gain"] >= 1.0)
    perr = phase_error(log["phase"][est], walk.phase[est])
    cmd, ideal = log["command"][full], log["ideal"][full]
    meaningful = np.abs(ideal) > 0.2 * PEAK_NM       # ignore near-zero crossings
    wrong = np.sign(cmd[meaningful]) != np.sign(ideal[meaningful])
    return {
        "phase_err_%": 100 * float(np.mean(perr)),
        "phase_err_p95_%": 100 * float(np.percentile(perr, 95)),
        "torque_rmse_Nm": float(np.sqrt(np.mean((cmd - ideal) ** 2))),
        "wrong_direction_%": 100 * float(np.mean(wrong)),
        "peak_timing_ms": peak_timing_ms(walk, log),
        "assist_on_%_of_walking": 100 * float(np.mean(log["walking"][walking_truth])),
    }


def peak_timing_ms(walk: SimWalk, log: dict) -> float:
    """Mean |time between the commanded and ideal flexion-torque peaks|, per full-assist stride."""
    ph = walk.phase
    starts = np.flatnonzero(np.isfinite(ph[1:]) & np.isfinite(ph[:-1]) & (np.diff(ph) < -0.5)) + 1
    errs = []
    for a, b in zip(starts[:-1], starts[1:]):
        if np.all(log["gain"][a:b] >= 1.0) and np.all(log["walking"][a:b]):
            errs.append(abs(np.argmax(log["command"][a:b]) - np.argmax(log["ideal"][a:b])) * DT * 1000)
    return float(np.mean(errs)) if errs else float("nan")


# ---- Figures ------------------------------------------------------------------------------------
def plot_all(walk: SimWalk, log: dict, m: dict, subject: str, out_dir: Path = PLOT_DIR) -> list:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    t = np.arange(len(walk.angle)) * DT
    good = log["walking"] & np.isfinite(walk.phase)
    paths = []

    # 1. Circle map
    fig, ax = plt.subplots(1, 2, figsize=(13, 5.5), layout="constrained")
    for a, (xs, ys, title, xl, yl) in zip(ax, [
            (log["theta"], log["theta_dot"], "Raw: thigh angle vs velocity", "angle (deg)", "velocity (deg/s)"),
            (log["x"], log["y"], "After normalize: angle vs integral (centered, / std)", "x", "y")]):
        sc = a.scatter(xs[good], ys[good], c=walk.phase[good], cmap="twilight", s=4, vmin=0, vmax=1)
        a.set_title(title)
        a.set_xlabel(xl)
        a.set_ylabel(yl)
        a.axhline(0, color="0.7", lw=0.5)
        a.axvline(0, color="0.7", lw=0.5)
        a.grid(alpha=0.2)
    ax[1].set_aspect("equal")
    fig.colorbar(sc, ax=ax, label="true gait phase (0 = heel strike)")
    fig.suptitle(f"{subject}: colours should go evenly around the circle")
    paths.append(_save(fig, out_dir / f"{subject}_1_circle.png", tight=False))

    # 2. Timeline
    fig, ax = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    ax[0].plot(t, walk.angle, color="0.4", lw=0.8)
    ax[0].set_ylabel("thigh angle (deg)")
    ax[1].plot(t, walk.phase, color="0.6", lw=2, label="true")
    ax[1].plot(t, log["phase"], color="C0", lw=0.9, label="estimated")
    ax[1].set_ylabel("gait phase")
    err = np.where(good, 100 * phase_error(np.nan_to_num(log["phase"]), np.nan_to_num(walk.phase)), np.nan)
    ax[2].plot(t, err, color="C3", lw=0.8)
    ax[2].axhline(5, color="0.5", ls="--", lw=1)
    ax[2].set_ylabel("phase error (%)")
    ax[3].plot(t, log["ideal"], color="0.6", lw=2, label="ideal (true phase)")
    ax[3].plot(t, log["command"], color="C2", lw=0.9, label="commanded (est. phase + safety)")
    ax[3].set_ylabel("torque (Nm, + = flexion)")
    ax[3].set_xlabel("time (s)")
    for a in ax:
        _shade(a, t, ~log["walking"], "0.85", "estimator: not walking / warming up")
        _shade(a, t, walk.stopped, "C1", "person actually stopped", alpha=0.15)
        a.grid(alpha=0.3)
    _label_speeds(ax[0], t, walk)
    ax[1].legend(loc="upper right", fontsize=8)
    ax[3].legend(loc="upper right", fontsize=8)
    handles, labels = ax[0].get_legend_handles_labels()
    ax[0].legend(handles, labels, loc="lower right", fontsize=8)
    fig.suptitle(f"{subject}: phase error {m['phase_err_%']:.1f}%  |  torque RMSE "
                 f"{m['torque_rmse_Nm']:.2f} Nm  |  wrong direction {m['wrong_direction_%']:.1f}%")
    paths.append(_save(fig, out_dir / f"{subject}_2_timeline.png"))

    # 3. Error vs where in the stride
    bins = np.linspace(0, 1, 41)
    centers = (bins[:-1] + bins[1:]) / 2
    signed = ((log["phase"] - walk.phase + 0.5) % 1 - 0.5)[good]
    which = np.clip(np.digitize(walk.phase[good], bins) - 1, 0, len(centers) - 1)
    mean_abs = [100 * np.mean(np.abs(signed[which == b])) for b in range(len(centers))]
    bias = [100 * np.mean(signed[which == b]) for b in range(len(centers))]
    fig, a = plt.subplots(figsize=(11, 5))
    a.bar(centers * 100, mean_abs, width=2.2, color="C3", alpha=0.6, label="mean |error|")
    a.plot(centers * 100, bias, color="C3", lw=2, label="bias (+ = estimate ahead)")
    a.axhline(0, color="0.5", lw=0.8)
    a.set_xlabel("true gait phase (% of stride, 0 = heel strike)")
    a.set_ylabel("phase error (% of stride)")
    a2 = a.twinx()
    ph = np.linspace(0, 1, 200)
    a2.plot(ph * 100, [_ideal_at(log, walk, p) for p in ph], color="0.3", ls="--", lw=1.2,
            label="ideal torque shape")
    a2.set_ylabel("torque (Nm)")
    a.axvline(60, color="0.5", ls=":", lw=1)
    a.text(60.5, a.get_ylim()[1] * 0.92, "≈ toe-off", fontsize=8, color="0.4")
    a.legend(loc="upper left", fontsize=8)
    a2.legend(loc="upper right", fontsize=8)
    a.set_title(f"{subject}: where in the stride the phase error happens")
    paths.append(_save(fig, out_dir / f"{subject}_3_error_by_phase.png"))

    # 4. Torque per stride
    full = good & (log["gain"] >= 1.0)
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    order = np.argsort(walk.phase[full])
    ax[0].scatter(walk.phase[full] * 100, log["command"][full], s=3, color="C2", alpha=0.4,
                  label="commanded, every tick")
    ax[0].plot(walk.phase[full][order] * 100, log["ideal"][full][order], color="k", lw=2, label="ideal")
    ax[0].set_xlabel("true gait phase (%)")
    ax[0].set_ylabel("torque (Nm, + = flexion)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[0].set_title("Torque vs true phase (full assist only)")
    ax[1].scatter(log["ideal"][full], log["command"][full], s=3, alpha=0.4, color="C2")
    lim = PEAK_NM * 1.1
    ax[1].plot([-lim, lim], [-lim, lim], color="k", lw=1)
    ax[1].axhline(0, color="0.7", lw=0.5)
    ax[1].axvline(0, color="0.7", lw=0.5)
    ax[1].set_xlabel("ideal torque (Nm)")
    ax[1].set_ylabel("commanded torque (Nm)")
    ax[1].set_aspect("equal")
    ax[1].set_title("Commanded vs ideal (on the line = perfect;\nwrong-sign quadrants = pushing the wrong way)")
    text = "\n".join(f"{k}: {v:.2f}" for k, v in m.items())
    ax[1].text(0.02, 0.98, text, transform=ax[1].transAxes, va="top", fontsize=8,
               bbox=dict(facecolor="white", alpha=0.8, edgecolor="0.8"))
    fig.suptitle(f"{subject}: is the torque mapped right?")
    paths.append(_save(fig, out_dir / f"{subject}_4_torque.png"))
    return paths


def _ideal_at(log, walk, p):
    k = np.nanargmin(np.abs(np.nan_to_num(walk.phase, nan=9) - p))
    return log["ideal"][k]


def _shade(ax, t, mask, color, label, alpha=0.5):
    edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
    for i, (a, b) in enumerate(zip(edges[::2], edges[1::2])):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color=color, alpha=alpha, lw=0,
                   label=label if i == 0 else None)


def _label_speeds(ax, t, walk):
    change = np.flatnonzero(np.diff(walk.speed) != 0) + 1
    for a, b in zip(np.concatenate([[0], change]), np.concatenate([change, [len(t)]])):
        s = walk.speed[a]
        ax.text((t[a] + t[b - 1]) / 2, ax.get_ylim()[1], "stop" if s == 0 else f"{s:g} m/s",
                ha="center", va="bottom", fontsize=9)


def _save(fig, path, tight=True):
    if tight:
        fig.tight_layout()
    fig.savefig(path, dpi=110)
    import matplotlib.pyplot as plt
    plt.close(fig)
    return path


# ---- Entry points -------------------------------------------------------------------------------
def run_subject(strides: Strides, subject: str, tmap: TorqueMap, plot: bool = True):
    lookup = fit_phase_lookup(strides.select(strides.subject != subject))
    walk = build_walk(strides, subject)
    log = simulate(walk, lookup, tmap)
    m = metrics(walk, log)
    paths = plot_all(walk, log, m, subject) if plot else []
    return m, paths


def main(argv):
    strides = load_walking()
    tmap = TorqueMap.load(peak_Nm=PEAK_NM)
    if "--all" in argv:
        rows = []
        for subject in np.unique(strides.subject):
            try:
                m, _ = run_subject(strides, subject, tmap, plot=False)
            except ValueError as e:
                print(f"  {subject}: skipped ({e})")
                continue
            rows.append(m)
            print(f"  {subject}: " + "  ".join(f"{k} {v:.2f}" for k, v in m.items()))
        print("MEAN:   " + "  ".join(f"{k} {np.nanmean([r[k] for r in rows]):.2f}" for k in rows[0]))
        return
    subject = next((a for a in argv if a.startswith("AB")), "AB05")
    m, paths = run_subject(strides, subject, tmap)
    print(subject, {k: round(v, 2) for k, v in m.items()})
    for p in paths:
        print("saved", p)


if __name__ == "__main__":
    main(sys.argv[1:])
