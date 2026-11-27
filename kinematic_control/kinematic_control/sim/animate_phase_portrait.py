"""Animation: a walking thigh, its phase portrait, the torque map, and estimated vs true.

What each panel shows (one held-out person, flat ground, 1.0 -> 1.2 -> 0.8 m/s):
    Leg          the thigh swinging (dataset angle), the exo at the hip, and the assist torque
                 as a curved arrow (green = commanded from ESTIMATED phase, grey = ideal from TRUE)
    Circle       the phase portrait: (angle, integral) normalized into a circle; the angle of
                 the dot around the circle is the estimated phase
    Torque map   how the map was made (grey: real hip torque, many strides of 9 OTHER people,
                 scaled to 2 Nm peak; black: the fitted map) and where we read it: estimated vs
                 true phase
    Bottom       gait phase and torque over time, estimated vs true, with the difference shaded

Run from kinematic_control/:
    python -m kinematic_control.sim.animate_phase_portrait            # MP4 (needs ffmpeg)
    python -m kinematic_control.sim.animate_phase_portrait --gif      # GIF instead
    python -m kinematic_control.sim.animate_phase_portrait AB03 --seconds 12
"""
import sys

import numpy as np

from kinematic_control.data_loader import PHASE, load_walking
from kinematic_control.estimation.phase_portrait.phase_portrait import fit_phase_lookup
from kinematic_control.replay import DT, phase_error
from kinematic_control.sim.replay_phase_portrait import (PEAK_NM, PLOT_DIR, STRIDE_M, build_walk,
                                                         simulate)
from kinematic_control.torque_map import TorqueMap

FPS = 25
SEGMENTS = [(1.0, 9), (1.2, 7), (0.8, 7)]
THIGH_M = 0.45          # drawn thigh length
WINDOW_S = 6.0          # seconds shown in the scrolling traces
TRAIL_S = 1.2           # dot trail on the circle


def main(argv):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.gridspec import GridSpec

    subject = next((a for a in argv if a.startswith("AB")), "AB05")
    seconds = float(argv[argv.index("--seconds") + 1]) if "--seconds" in argv else None
    gif = "--gif" in argv

    strides = load_walking()
    others = strides.select(strides.subject != subject)
    tmap = TorqueMap.load(peak_Nm=PEAK_NM)
    lookup = fit_phase_lookup(others)
    walk = build_walk(strides, subject, segments=SEGMENTS)
    log = simulate(walk, lookup, tmap)

    n = len(walk.angle) if seconds is None else min(len(walk.angle), int(seconds / DT))
    t = np.arange(len(walk.angle)) * DT
    step = max(1, int(round(1 / (FPS * DT))))          # data samples per video frame (real time)
    frames = range(0, n, step)
    ok = log["walking"] & np.isfinite(walk.phase)
    perr = np.where(ok, 100 * phase_error(np.nan_to_num(log["phase"]), np.nan_to_num(walk.phase)), np.nan)

    # Torque map background: real hip torque of OTHER people, flat, 1.0-1.2 m/s, same scaling as the map
    m = (others.incline == 0) & (others.speed >= 1.0) & (others.leg == 1)
    pick = np.random.default_rng(0).choice(np.flatnonzero(m), size=min(80, m.sum()), replace=False)
    real_curves = others.torque[pick] * PEAK_NM * tmap.scale
    real_curves = real_curves[np.max(np.abs(real_curves), axis=1) < 1.6 * PEAK_NM]   # drop glitchy strides
    ph_grid = np.linspace(0, 1, 300, endpoint=False)
    map_curve = PEAK_NM * tmap.scale * tmap.normal_torque(ph_grid, np.full(300, STRIDE_M), np.zeros(300))

    # ---- Layout -----------------------------------------------------------------------------
    fig = plt.figure(figsize=(16, 9.5))
    gs = GridSpec(4, 3, figure=fig, height_ratios=[1, 1, 0.55, 0.55], hspace=0.55, wspace=0.28)
    ax_leg = fig.add_subplot(gs[0:2, 0])
    ax_c = fig.add_subplot(gs[0:2, 1])
    ax_map = fig.add_subplot(gs[0:2, 2])
    ax_ph = fig.add_subplot(gs[2, :])
    ax_tq = fig.add_subplot(gs[3, :], sharex=ax_ph)

    # Leg
    ax_leg.set_xlim(-0.45, 0.45)
    ax_leg.set_ylim(-0.62, 0.18)
    ax_leg.set_aspect("equal")
    ax_leg.axis("off")
    ax_leg.set_title("Leg + exo (walking to the right)")
    ax_leg.add_patch(plt.Rectangle((-0.12, 0.0), 0.24, 0.12, color="0.75"))       # pelvis
    ax_leg.text(0, 0.06, "pelvis", ha="center", va="center", fontsize=8)
    thigh, = ax_leg.plot([], [], color="tan", lw=14, solid_capstyle="round")
    link, = ax_leg.plot([], [], color="0.25", lw=3)
    cuff, = ax_leg.plot([], [], "s", color="0.25", ms=12)
    ax_leg.add_patch(plt.Circle((0, 0), 0.035, color="0.2", zorder=5))           # motor
    arc_cmd, = ax_leg.plot([], [], color="C2", lw=4)
    arc_ideal, = ax_leg.plot([], [], color="0.55", lw=2, ls="--")
    head_cmd = ax_leg.annotate("", xy=(0, 0), xytext=(0, 0),
                               arrowprops=dict(arrowstyle="-|>", color="C2", lw=3, mutation_scale=22))
    leg_txt = ax_leg.text(0, -0.58, "", ha="center", fontsize=10, family="monospace")
    ax_leg.plot([], [], color="C2", lw=4, label="torque from ESTIMATED phase")
    ax_leg.plot([], [], color="0.55", lw=2, ls="--", label="torque from TRUE phase")
    ax_leg.legend(loc="upper right", fontsize=7, bbox_to_anchor=(1.02, 1.02))

    # Circle
    good = ok.copy()
    good[n:] = False
    ax_c.scatter(log["x"][good], log["y"][good], c=walk.phase[good], cmap="twilight", s=2,
                 vmin=0, vmax=1, alpha=0.25)
    ax_c.axhline(0, color="0.8", lw=0.6)
    ax_c.axvline(0, color="0.8", lw=0.6)
    ax_c.set_xlim(-2.6, 2.6)
    ax_c.set_ylim(-2.6, 2.6)
    ax_c.set_aspect("equal")
    ax_c.set_xlabel("x = thigh angle (centered, / std)")
    ax_c.set_ylabel("y = integral of angle (centered, / std)")
    ax_c.set_title("Phase portrait: angle around the circle = phase\n(background coloured by TRUE phase)")
    trail, = ax_c.plot([], [], color="C0", lw=1.5)
    spoke, = ax_c.plot([], [], color="C0", lw=1)
    dot, = ax_c.plot([], [], "o", color="C0", ms=10)
    c_txt = ax_c.text(-2.5, 2.35, "", fontsize=9, va="top", family="monospace")

    # Torque map
    for curve in real_curves:
        ax_map.plot(PHASE * 100, curve, color="0.8", lw=0.6, zorder=1)
    ax_map.plot(ph_grid * 100, map_curve, color="k", lw=2.5, zorder=2, label="torque map (fit)")
    ax_map.plot([], [], color="0.8", lw=2, label="real hip torque, other people (scaled)")
    ax_map.axhline(0, color="0.6", lw=0.6)
    ax_map.set_xlim(0, 100)
    ax_map.set_xlabel("gait phase (%)  0 = heel strike")
    ax_map.set_ylabel("assist torque (Nm, + = flexion)")
    ax_map.set_title(f"Torque map: phase → torque\n(dataset shape, scaled to {PEAK_NM:g} Nm peak)")
    vl_true = ax_map.axvline(0, color="0.5", ls="--", lw=1.5)
    vl_est = ax_map.axvline(0, color="C2", lw=1.5)
    pt_true, = ax_map.plot([], [], "o", color="0.5", ms=9, zorder=4, label="read at TRUE phase")
    pt_est, = ax_map.plot([], [], "o", color="C2", ms=9, zorder=5, label="read at ESTIMATED phase")
    ax_map.legend(loc="lower left", fontsize=7)

    # Traces
    ph_true, = ax_ph.plot([], [], color="0.55", lw=2.5, label="true")
    ph_est, = ax_ph.plot([], [], color="C0", lw=1.2, label="estimated")
    ax_ph.set_ylabel("gait phase")
    ax_ph.set_ylim(-0.05, 1.05)
    ax_ph.legend(loc="upper left", fontsize=8, ncol=2)
    tq_ideal, = ax_tq.plot([], [], color="0.55", lw=2.5, label="ideal (true phase)")
    tq_cmd, = ax_tq.plot([], [], color="C2", lw=1.2, label="commanded (estimated phase + safety)")
    ax_tq.fill_between([], [], [], color="C3", alpha=0.25, label="difference")
    ax_tq.set_ylim(-1.3 * PEAK_NM * 0.6, 1.3 * PEAK_NM * 0.8)
    diff = [None]
    ax_tq.set_ylabel("torque (Nm)")
    ax_tq.set_xlabel("time (s)")
    ax_tq.legend(loc="upper left", fontsize=8, ncol=3)
    for a in (ax_ph, ax_tq):
        a.grid(alpha=0.3)
    _shade(ax_ph, t[:n], ~log["walking"][:n])
    _shade(ax_tq, t[:n], ~log["walking"][:n])
    ax_ph.text(0.2, 0.5, "warming up\n(torque off)", fontsize=8, color="0.4", va="center")
    now_ph = ax_ph.axvline(0, color="k", lw=1)
    now_tq = ax_tq.axvline(0, color="k", lw=1)
    title = fig.suptitle("", fontsize=13)

    # ---- One frame --------------------------------------------------------------------------
    def draw(k):
        th = np.radians(walk.angle[k])
        knee = np.array([THIGH_M * np.sin(th), -THIGH_M * np.cos(th)])
        thigh.set_data([0, knee[0]], [0, knee[1]])
        cuff_pt = 0.55 * knee
        link.set_data([0, cuff_pt[0]], [0, cuff_pt[1]])
        cuff.set_data([cuff_pt[0]], [cuff_pt[1]])
        _arc(arc_cmd, head_cmd, th, log["command"][k], 0.30)
        _arc(arc_ideal, None, th, log["ideal"][k], 0.36)
        leg_txt.set_text(f"commanded {log['command'][k]:+5.2f} Nm   ideal {log['ideal'][k]:+5.2f} Nm")

        a = max(0, k - int(TRAIL_S / DT))
        if np.isfinite(log["x"][k]):
            trail.set_data(log["x"][a:k + 1], log["y"][a:k + 1])
            dot.set_data([log["x"][k]], [log["y"][k]])
            spoke.set_data([0, log["x"][k]], [0, log["y"][k]])
        est, true = log["phase"][k], walk.phase[k]
        c_txt.set_text(f"estimated {est * 100:5.1f}%\ntrue      {true * 100:5.1f}%" if np.isfinite(est)
                       else "warming up…")

        if np.isfinite(est):
            vl_est.set_xdata([est * 100])
            pt_est.set_data([est * 100], [log["desired"][k]])
        vl_true.set_xdata([true * 100])
        pt_true.set_data([true * 100], [log["ideal"][k]])

        ph_true.set_data(t[:k + 1], walk.phase[:k + 1])
        ph_est.set_data(t[:k + 1], log["phase"][:k + 1])
        tq_ideal.set_data(t[:k + 1], log["ideal"][:k + 1])
        tq_cmd.set_data(t[:k + 1], log["command"][:k + 1])
        if diff[0] is not None:
            diff[0].remove()
        diff[0] = ax_tq.fill_between(t[:k + 1], log["ideal"][:k + 1], log["command"][:k + 1],
                                     where=log["walking"][:k + 1], color="C3", alpha=0.25)
        for line in (now_ph, now_tq):
            line.set_xdata([t[k]])
        left = max(0.0, t[k] - WINDOW_S * 0.8)
        ax_ph.set_xlim(left, left + WINDOW_S)
        e = perr[k]
        title.set_text(f"{subject} (held out) · {walk.speed[k]:g} m/s · t = {t[k]:4.1f} s · "
                       + (f"phase error {e:4.1f}%" if np.isfinite(e) else "warming up"))
        return []

    anim = animation.FuncAnimation(fig, draw, frames=frames, interval=1000 / FPS, blit=False)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    path = PLOT_DIR / f"{subject}_animation.{'gif' if gif else 'mp4'}"
    writer = animation.PillowWriter(fps=FPS) if gif else animation.FFMpegWriter(fps=FPS, bitrate=2500)
    anim.save(path, writer=writer, dpi=80 if gif else 100)
    print("saved", path)


def _arc(line, head, thigh_rad, torque, radius):
    """Curved arrow around the hip: length ~ |torque|, direction = sign (+ = flexion = forward)."""
    span = np.clip(torque / PEAK_NM, -1, 1) * np.radians(70)
    if abs(span) < np.radians(2):
        line.set_data([], [])
        if head is not None:
            head.set_visible(False)
        return
    start = -np.pi / 2 + thigh_rad                     # starts at the thigh, sweeps the way it pushes
    ang = np.linspace(start, start + span, 30)
    xs, ys = radius * np.cos(ang), radius * np.sin(ang)
    line.set_data(xs, ys)
    if head is not None:
        head.set_visible(True)
        head.xy = (xs[-1], ys[-1])
        head.set_position((xs[-3], ys[-3]))


def _shade(ax, t, mask):
    edges = np.flatnonzero(np.diff(np.concatenate([[0], mask.astype(int), [0]])))
    for a, b in zip(edges[::2], edges[1::2]):
        ax.axvspan(t[a], t[min(b, len(t) - 1)], color="0.88", lw=0)


if __name__ == "__main__":
    main(sys.argv[1:])
