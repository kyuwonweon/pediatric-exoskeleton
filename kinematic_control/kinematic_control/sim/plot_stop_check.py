"""Zoom on the stop in the replay walk: does the stop check turn torque off quickly?

Compares, on the same walk and the same held-out person:
    old: only the backup rule (no thigh turnaround for 1.5 s)
    new: + the fast angle rule (thigh angle stays within 12 % of last swing for 0.4 s)
and shows why a velocity rule was dropped (sensor noise).

Run from kinematic_control/:  python -m kinematic_control.sim.plot_stop_check [AB05]
"""
import sys

import numpy as np

from kinematic_control.data_loader import load_walking
from kinematic_control.estimation.phase_portrait import PhasePortrait
from kinematic_control.estimation.phase_portrait.phase_portrait import fit_phase_lookup
from kinematic_control.replay import DT
from kinematic_control.safety import Safety
from kinematic_control.sim.replay_phase_portrait import PEAK_NM, PLOT_DIR, STRIDE_M, build_walk
from kinematic_control.torque_map import TorqueMap


def run(walk, lookup, tmap, **settings):
    """Chain with per-tick logging of the stop check's inside numbers."""
    pp = PhasePortrait(lookup=lookup, **settings)
    safety = Safety(dt=DT)
    safety.enable()
    velocity = np.gradient(walk.angle, DT)
    n = len(walk.angle)
    out = {k: np.full(n, np.nan) for k in ("phase", "torque", "band", "limit", "theta_dot")}
    out["stopped"] = np.zeros(n, bool)
    for k in range(n):
        ph = pp.step(walk.angle[k])
        if len(pp.recent) == pp.recent.maxlen:
            out["band"][k] = max(pp.recent) - min(pp.recent)
            out["limit"][k] = pp.still_frac * pp.stride_size
        if len(pp.angles) == pp.angles.maxlen:
            out["theta_dot"][k] = pp.weights[1] @ np.asarray(pp.angles)
        out["stopped"][k] = not pp.walking
        if ph is not None:
            out["phase"][k] = ph
        desired = tmap.command(ph, 0.0, STRIDE_M, 0.0) if ph is not None else 0.0
        out["torque"][k] = safety.step(desired, pp.walking, walk.angle[k], velocity[k])
    return out


def main(argv):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    subject = next((a for a in argv if a.startswith("AB")), "AB05")
    strides = load_walking()
    tmap = TorqueMap.load(peak_Nm=PEAK_NM)
    lookup = fit_phase_lookup(strides.select(strides.subject != subject))
    walk = build_walk(strides, subject)
    new = run(walk, lookup, tmap)
    old = run(walk, lookup, tmap, still_frac=0.0)      # fast rule can never fire = backup only

    t = np.arange(len(walk.angle)) * DT
    stop = np.flatnonzero(walk.stopped)
    t_stop, t_rest, t_go = t[stop[0]], t[stop[0]] + 0.5, t[stop[-1]] - 0.5   # 0.5 s ramps
    view = (t > t_stop - 2.5) & (t < t_go + 3.5)

    def first_off(o):
        k = np.flatnonzero(o["stopped"] & (t >= t_stop))
        return t[k[0]] if len(k) else np.nan

    def torque_off(o):   # last tick with torque during the stop
        k = np.flatnonzero((np.abs(o["torque"]) > 0.05) & walk.stopped)
        return t[k[-1]] if len(k) else t_stop
    t_new, t_old = first_off(new), first_off(old)

    fig, ax = plt.subplots(5, 1, figsize=(13, 13), sharex=True)
    ax[0].plot(t[view], walk.angle[view], color="0.3", lw=1)
    ax[0].set_ylabel("thigh angle (deg)")
    ax[0].set_title("1. The person stops: thigh moves to upright (0.5 s), stands 3 s, starts again")

    ax[1].plot(t[view], new["band"][view], color="C0", lw=1.2, label="angle band over last 0.4 s (max - min)")
    ax[1].plot(t[view], new["limit"][view], color="C3", ls="--", lw=1.2, label="threshold = 12 % of last swing")
    ax[1].fill_between(t[view], 0, new["band"][view], where=(new["band"] < new["limit"])[view],
                       color="C3", alpha=0.25, label="below threshold -> 'still'")
    ax[1].set_ylabel("deg")
    ax[1].set_ylim(0, 1.2 * np.nanmax(new["band"][view]))
    ax[1].legend(loc="upper right", fontsize=8)
    ax[1].set_title("2. NEW rule: when walking the band is wide; standing it collapses under the threshold")

    vel_thr = 17.0   # what the dropped velocity rule used here (0.2 x ~85 deg/s stride std)
    ax[2].plot(t[view], np.abs(new["theta_dot"][view]), color="C1", lw=0.8, label="|thigh velocity|")
    ax[2].axhline(vel_thr, color="C3", ls="--", lw=1.2, label=f"velocity threshold ({vel_thr:.0f} deg/s)")
    ax[2].set_ylim(0, 80)
    ax[2].set_ylabel("deg/s")
    ax[2].legend(loc="upper right", fontsize=8)
    ax[2].set_title("3. DROPPED velocity rule: while standing, sensor noise keeps crossing the threshold, "
                    "so '0.3 s in a row below it' never happens")

    ax[3].fill_between(t[view], 1.1, 1.9, where=old["stopped"][view], color="C1", alpha=0.6, step="mid",
                       label="old: backup rule only (1.5 s)")
    ax[3].fill_between(t[view], 0.1, 0.9, where=new["stopped"][view], color="C0", alpha=0.6, step="mid",
                       label="new: + fast angle rule")
    ax[3].set_yticks([0.5, 1.5], ["new", "old"])
    ax[3].set_ylim(0, 2)
    ax[3].legend(loc="upper right", fontsize=8)
    ax[3].set_title("4. 'Not walking' flag (torque gate)")

    ax[4].plot(t[view], old["torque"][view], color="C1", lw=1.4, label="old")
    ax[4].plot(t[view], new["torque"][view], color="C0", lw=1.4, label="new")
    ax[4].axhline(0, color="0.6", lw=0.6)
    ax[4].set_ylabel("commanded torque (Nm)")
    ax[4].set_xlabel("time (s)")
    ax[4].legend(loc="upper right", fontsize=8)
    ax[4].set_title(f"5. Torque: off {torque_off(old) - t_rest:.2f} s after the thigh rests (old) vs "
                    f"{torque_off(new) - t_rest:.2f} s (new)")

    for a in ax:
        a.axvspan(t_stop, t_rest, color="C1", alpha=0.08)
        a.axvspan(t_rest, t_go, color="C1", alpha=0.18)
        a.axvline(t_new, color="C0", lw=1, ls=":")
        a.axvline(t_old, color="C1", lw=1, ls=":")
        a.grid(alpha=0.25)
    ymax = ax[0].get_ylim()[1]
    ax[0].text(t_stop, ymax, "stop starts", fontsize=8, va="bottom", ha="right")
    ax[0].text(t_rest, ymax * 0.92, " thigh at rest", fontsize=8, va="top")
    ax[0].text(t_new, ymax * 0.8, f" new: +{t_new - t_rest:.2f} s", color="C0", fontsize=8)
    ax[0].text(t_old, ymax * 0.6, f" old: +{t_old - t_rest:.2f} s", color="C1", fontsize=8)
    fig.suptitle(f"{subject}: stop check", fontsize=13)
    fig.tight_layout()
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    path = PLOT_DIR / f"{subject}_5_stop_check.png"
    fig.savefig(path, dpi=110)
    print("saved", path)
    print(f"stop detected after thigh at rest: new {t_new - t_rest:+.2f} s, old {t_old - t_rest:+.2f} s")


if __name__ == "__main__":
    main(sys.argv[1:])
