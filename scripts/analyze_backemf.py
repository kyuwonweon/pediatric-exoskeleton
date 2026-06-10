#!/usr/bin/env python3
"""
Back-EMF + acceleration feedforward validation.

Supports two test types (auto-detected):
  manual   — test_backemf_manual.py   : slot 11 is always 1.0/2.0/3.0
  hardware — test_backemf_hardware.py : slot 11 is 1.0/2.0/3.0 during pauses,
                                        commanded position during sweeps

Plots:
  1. Binned mean current vs velocity  (Phase 1 vs 2) — back-EMF gain
  2. Binned mean current vs accel     (Phase 2 vs 3) — acc FF gain
  3. _tau_acc_ff vs acceleration      (Phase 3)      — acc FF math check
  4. Actual current vs _tau_commanded (all phases)   — model accuracy
  5. Position tracking error vs time  (all phases)   — hardware test only

Usage:
    python3 analyze_backemf.py <bag_folder>
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── message indices ───────────────────────────────────────────────────────────
# 0  timestamp    3  acceleration    6  _tau_acc_ff     9  temperature
# 1  position     4  _tau_commanded  7  _tau_backemf_ff 10 _tau_acc_ff
# 2  velocity     5  actual torque   8  current         11 phase marker / q_des
IDX_TIMESTAMP   = 0
IDX_POSITION    = 1
IDX_VELOCITY    = 2
IDX_ACCEL       = 3
IDX_TAU_CMD     = 4
IDX_BACKEMF_FF  = 7
IDX_CURRENT     = 8
IDX_ACC_FF      = 10
IDX_PHASE       = 11

VEL_THRESHOLD = 5.0   # deg/s — exclude near-still samples
MARKER_VALUES = {1.0, 2.0, 3.0}


def read_bag(bag_path: str) -> np.ndarray:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', '')
    )
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    rows = []
    while reader.has_next():
        topic, msg_bytes, _ = reader.read_next()
        if topic == '/robot_state_CubeMars':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(msg_bytes, msg_type)
            rows.append(list(msg.data))
    return np.array(rows, dtype=np.float64)


def is_hardware_test(data: np.ndarray) -> bool:
    """Hardware test has commanded sine in slot 11 during sweeps — not just 1/2/3."""
    marker = data[:, IDX_PHASE]
    sweep_samples = ~np.isin(np.round(marker, 4), list(MARKER_VALUES) + [0.0])
    return sweep_samples.sum() > 20


def split_phases(data: np.ndarray):
    """
    Manual test:  slot 11 is always 1/2/3 → filter by exact value.
    Hardware test: slot 11 is 1/2/3 during pauses, q_des during sweeps →
                   split temporally at the first appearance of each marker.
    Returns (p1, p2, p3, test_type) where test_type is 'manual' or 'hardware'.
    """
    marker = data[:, IDX_PHASE]

    if not is_hardware_test(data):
        p1 = data[marker == 1.0]
        p2 = data[marker == 2.0]
        p3 = data[marker == 3.0]
        if len(p1) == 0 or len(p2) == 0 or len(p3) == 0:
            print(f"WARNING: phase marker missing. Counts: p1={len(p1)} p2={len(p2)} p3={len(p3)}")
        return p1, p2, p3, 'manual'

    # Hardware: find first index of each marker to get phase boundaries
    idx1 = np.where(marker == 1.0)[0]
    idx2 = np.where(marker == 2.0)[0]
    idx3 = np.where(marker == 3.0)[0]

    if len(idx1) == 0 or len(idx2) == 0 or len(idx3) == 0:
        print("WARNING: phase markers not found in hardware bag.")
        return data, data, data, 'hardware'

    start1, start2, start3 = idx1[0], idx2[0], idx3[0]
    p1 = data[start1:start2]
    p2 = data[start2:start3]
    p3 = data[start3:]
    return p1, p2, p3, 'hardware'


def get_sweep_data(phase: np.ndarray):
    """Extract only sweep samples (slot 11 = commanded position, not marker)."""
    marker = phase[:, IDX_PHASE]
    sweep = ~np.isin(np.round(marker, 4), list(MARKER_VALUES) + [0.0])
    return phase[sweep]


def moving(phase: np.ndarray) -> np.ndarray:
    if len(phase) == 0:
        return phase
    return phase[np.abs(phase[:, IDX_VELOCITY]) > VEL_THRESHOLD]


def bin_mean(x, y, bins):
    centers = 0.5 * (bins[:-1] + bins[1:])
    means, stds = [], []
    for lo, hi in zip(bins[:-1], bins[1:]):
        mask = (x >= lo) & (x < hi)
        if mask.sum() > 5:
            means.append(np.mean(y[mask]))
            stds.append(np.std(y[mask]) / np.sqrt(mask.sum()))
        else:
            means.append(np.nan)
            stds.append(np.nan)
    return centers, np.array(means), np.array(stds)


def linear_fit(x, y):
    mask = ~np.isnan(x) & ~np.isnan(y)
    if mask.sum() < 2:
        return np.nan, np.nan
    return np.polyfit(x[mask], y[mask], 1)


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "backemf_manual_test"
    print(f"Reading bag: {bag_path}")
    data = read_bag(bag_path)
    print(f"  Total samples: {len(data)}")
    if len(data) == 0:
        print("ERROR: bag is empty.")
        return

    p1_raw, p2_raw, p3_raw, test_type = split_phases(data)
    print(f"  Test type: {test_type}")

    # moving-filtered data for current plots
    p1 = moving(p1_raw)
    p2 = moving(p2_raw)
    p3 = moving(p3_raw)
    print(f"  Phase 1 (FF off):      {len(p1)} moving samples")
    print(f"  Phase 2 (Back-EMF FF): {len(p2)} moving samples")
    print(f"  Phase 3 (+ Acc FF):    {len(p3)} moving samples")

    if len(p1) == 0 or len(p2) == 0:
        print("ERROR: Phase 1 or 2 empty — check bag.")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Feedforward Validation', fontsize=14)

    # ── Plot 1: current vs velocity (Phase 1 vs 2) ───────────────────────────
    ax = axes[0, 0]
    vel_bins = np.linspace(-150, 150, 16)
    c1, m1, s1 = bin_mean(p1[:, IDX_VELOCITY], p1[:, IDX_CURRENT], vel_bins)
    c2, m2, s2 = bin_mean(p2[:, IDX_VELOCITY], p2[:, IDX_CURRENT], vel_bins)
    valid1, valid2 = ~np.isnan(m1), ~np.isnan(m2)
    ax.errorbar(c1[valid1], m1[valid1], yerr=s1[valid1], fmt='o-', color='steelblue', label='Phase 1 — FF off')
    ax.errorbar(c2[valid2], m2[valid2], yerr=s2[valid2], fmt='s-', color='tomato',    label='Phase 2 — Back-EMF FF')
    k1, _ = linear_fit(c1[valid1], m1[valid1])
    k2, _ = linear_fit(c2[valid2], m2[valid2])
    pct = (abs(k1) - abs(k2)) / abs(k1) * 100 if k1 != 0 else 0
    ax.set_title(f'Current vs Velocity\nslope: {k1:.4f} → {k2:.4f} A/(deg/s)  ({pct:.1f}% reduction)')
    ax.set_xlabel('Velocity (deg/s)')
    ax.set_ylabel('Mean current (A)')
    ax.legend()
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.axvline(0, color='k', lw=0.5, ls='--')
    ax.grid(True, alpha=0.3)

    # ── Plot 2: current vs acceleration (Phase 2 vs 3) ───────────────────────
    ax = axes[0, 1]
    if len(p3) > 0:
        acc_bins = np.linspace(-800, 800, 16)
        c2a, m2a, s2a = bin_mean(p2[:, IDX_ACCEL], p2[:, IDX_CURRENT], acc_bins)
        c3a, m3a, s3a = bin_mean(p3[:, IDX_ACCEL], p3[:, IDX_CURRENT], acc_bins)
        valid2a, valid3a = ~np.isnan(m2a), ~np.isnan(m3a)
        ax.errorbar(c2a[valid2a], m2a[valid2a], yerr=s2a[valid2a], fmt='o-', color='steelblue', label='Phase 2 — Acc FF off')
        ax.errorbar(c3a[valid3a], m3a[valid3a], yerr=s3a[valid3a], fmt='s-', color='tomato',    label='Phase 3 — Acc FF on')
        k2a, _ = linear_fit(c2a[valid2a], m2a[valid2a])
        k3a, _ = linear_fit(c3a[valid3a], m3a[valid3a])
        pct_a = (abs(k2a) - abs(k3a)) / abs(k2a) * 100 if k2a != 0 else 0
        ax.set_title(f'Current vs Acceleration\nslope: {k2a:.5f} → {k3a:.5f} A/(deg/s²)  ({pct_a:.1f}% reduction)')
    else:
        ax.set_title('Current vs Acceleration\n(no Phase 3 data)')
    ax.set_xlabel('Acceleration (deg/s²)')
    ax.set_ylabel('Mean current (A)')
    ax.legend()
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.axvline(0, color='k', lw=0.5, ls='--')
    ax.grid(True, alpha=0.3)

    # ── Plot 3: _tau_acc_ff vs acceleration (Phase 3) ────────────────────────
    ax = axes[1, 0]
    if len(p3) > 0:
        acc_vals = p3[:, IDX_ACCEL]
        acc_ff   = p3[:, IDX_ACC_FF]
        ax.scatter(acc_vals, acc_ff, s=3, alpha=0.3, color='tomato')
        k, b = np.polyfit(acc_vals, acc_ff, 1)
        x_line = np.linspace(acc_vals.min(), acc_vals.max(), 100)
        ax.plot(x_line, k * x_line + b, 'k-', lw=1.5, label=f'fit: slope={k:.5f}')
        ax.set_title(f'_tau_acc_ff vs Acceleration (Phase 3)\nslope={k:.5f} A/(deg/s²) — should be linear through origin')
    else:
        ax.set_title('_tau_acc_ff vs Acceleration\n(no Phase 3 data)')
    ax.set_xlabel('Acceleration (deg/s²)')
    ax.set_ylabel('_tau_acc_ff (A)')
    ax.axhline(0, color='k', lw=0.5, ls='--')
    ax.axvline(0, color='k', lw=0.5, ls='--')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── Plot 4: actual current vs _tau_commanded (all phases) ────────────────
    ax = axes[1, 1]
    for phase, color, label in [
        (p1, 'steelblue', 'Phase 1'),
        (p2, 'orange',    'Phase 2'),
        (p3, 'tomato',    'Phase 3'),
    ]:
        if len(phase) > 0:
            ax.scatter(phase[:, IDX_TAU_CMD], phase[:, IDX_CURRENT],
                       s=3, alpha=0.3, color=color, label=label)
    all_cmd = np.concatenate([p[:, IDX_TAU_CMD] for p in [p1, p2, p3] if len(p) > 0])
    lim = np.percentile(np.abs(all_cmd), 99) * 1.1
    ax.plot([-lim, lim], [-lim, lim], 'k--', lw=1.2, label='y = x (perfect model)')
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel('_tau_commanded (A)')
    ax.set_ylabel('Actual current (A)')
    ax.set_title('Actual current vs Commanded\n(points on y=x = perfect model)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # ── Plots 5-10: tracking error + position per phase (hardware only) ────────
    if test_type == 'hardware':
        fig2, axes2 = plt.subplots(3, 2, figsize=(14, 12))
        fig2.suptitle('Position Tracking — per phase', fontsize=14)

        phases_hw = [
            (p1_raw, 'steelblue', 'Phase 1 — FF off'),
            (p2_raw, 'orange',    'Phase 2 — Back-EMF FF'),
            (p3_raw, 'tomato',    'Phase 3 — + Acc FF'),
        ]

        for row, (phase_raw, color, label) in enumerate(phases_hw):
            ax_err = axes2[row, 0]
            ax_pos = axes2[row, 1]

            sweep = get_sweep_data(phase_raw)
            if len(sweep) == 0:
                ax_err.set_title(f'{label}\n(no sweep data)')
                ax_pos.set_title(f'{label}\n(no sweep data)')
                continue

            t        = sweep[:, IDX_TIMESTAMP] - sweep[0, IDX_TIMESTAMP]
            q_des    = sweep[:, IDX_PHASE]
            q_actual = sweep[:, IDX_POSITION]
            error    = q_des - q_actual
            rms_err  = np.sqrt(np.mean(error ** 2))
            max_err  = np.max(np.abs(error))

            ax_err.plot(t, error, lw=1, color=color)
            ax_err.axhline(0, color='k', lw=0.5, ls='--')
            ax_err.set_ylabel('Error (deg)')
            ax_err.set_title(f'{label}\nTracking error  RMS={rms_err:.2f}°  max={max_err:.2f}°')
            ax_err.grid(True, alpha=0.3)

            ax_pos.plot(t, q_des,    lw=1.5, ls='--', color='k',   label='Commanded')
            ax_pos.plot(t, q_actual, lw=1,   color=color,           label='Actual')
            ax_pos.set_ylabel('Position (deg)')
            ax_pos.set_title(f'{label}\nCommanded vs Actual')
            ax_pos.legend()
            ax_pos.grid(True, alpha=0.3)

        for ax in axes2[-1, :]:
            ax.set_xlabel('Time (s)')

        fig2.tight_layout()
        fig2.savefig('tracking_error.png', dpi=150)
        print("Saved: tracking_error.png")

    plt.tight_layout()
    plt.savefig('feedforward_validation.png', dpi=150)
    print("Saved: feedforward_validation.png")
    plt.show()


if __name__ == '__main__':
    main()
