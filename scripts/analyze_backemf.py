#!/usr/bin/env python3
"""
Back-EMF feedforward validation analysis.

Reads a ROS2 bag from the hardware test and produces torque and current
vs velocity plots for Phase 1 (FF off) and Phase 2 (FF on).

Usage:
    python3 analyze_backemf.py <path_to_bag_folder>            # PVT trajectory mode
    python3 analyze_backemf.py <path_to_bag_folder> --manual   # manual rotation mode

In --manual mode the x-axis uses measured velocity instead of commanded
velocity derived from q_des, and the moving-sample filter uses
|measured_velocity| > VEL_THRESHOLD_DEG_S instead of |q_des| > 1 deg.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ── trajectory parameters (must match test script) ────────────────────────────
AMP       = 30.0    # deg
FREQ_HZ   = 0.25    # Hz
BACKEMF_FF_GAIN     = 1.0   # A / (rev/s)  — empirically tuned (Ke/R ≈ 1.3 causes over-compensation)
VEL_THRESHOLD_DEG_S = 5.0   # deg/s — manual mode: ignore samples where motor is barely moving

# ── data indices in /robot_state_CubeMars ─────────────────────────────────────
IDX_TIMESTAMP  = 0
IDX_POSITION   = 1
IDX_VELOCITY   = 2   # measured velocity deg/s (120 Hz filtered)
IDX_ACTUAL_TRQ = 5
IDX_FF_TERM    = 7
IDX_CURRENT    = 8   # quadrature current A
IDX_Q_DES      = 11


def read_bag(bag_path: str):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

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


def compute_commanded_velocity(q_des: np.ndarray, timestamps: np.ndarray) -> np.ndarray:
    """Differentiate q_des numerically to get clean commanded velocity in deg/s."""
    qdot = np.gradient(q_des, timestamps)
    return qdot


def split_phases(data: np.ndarray, manual: bool = False):
    """
    Split data into Phase 1 (FF off) and Phase 2 (FF on)
    by detecting when the FF term (data[7]) becomes non-zero.

    manual=True: filter by |measured_velocity| > threshold instead of |q_des| > 1.
    """
    ff = data[:, IDX_FF_TERM]
    ff_active = np.abs(ff) > 1e-4

    transition = np.where(np.diff(ff_active.astype(int)) > 0)[0]
    if len(transition) == 0:
        print("WARNING: Could not find Phase 1 → Phase 2 transition. "
              "Is FF actually being toggled?")
        return data, data

    split = transition[0] + 1
    phase1_data = data[:split]
    phase2_data = data[split:]

    if manual:
        p1_moving = np.abs(phase1_data[:, IDX_VELOCITY]) > VEL_THRESHOLD_DEG_S
        p2_moving = np.abs(phase2_data[:, IDX_VELOCITY]) > VEL_THRESHOLD_DEG_S
    else:
        p1_moving = np.abs(phase1_data[:, IDX_Q_DES]) > 1.0
        p2_moving = np.abs(phase2_data[:, IDX_Q_DES]) > 1.0

    return phase1_data[p1_moving], phase2_data[p2_moving]


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else "backemf_hw_test"
    manual   = '--manual' in sys.argv

    print(f"Reading bag: {bag_path}  (mode: {'manual' if manual else 'PVT'})")
    data = read_bag(bag_path)
    print(f"  Total samples: {len(data)}")
    if data.ndim < 2 or len(data) == 0:
        print("ERROR: bag is empty — start the bag recording before running the test.")
        return

    if manual:
        # Use measured velocity directly — no commanded velocity needed.
        data_with_vel = data
        phase1, phase2 = split_phases(data_with_vel, manual=True)
        vel1 = phase1[:, IDX_VELOCITY]   # measured velocity deg/s
        vel2 = phase2[:, IDX_VELOCITY]
        vel_label = 'Measured velocity (deg/s)'
    else:
        timestamps    = data[:, IDX_TIMESTAMP]
        q_des         = data[:, IDX_Q_DES]
        qdot_cmd      = compute_commanded_velocity(q_des, timestamps)
        data_with_vel = np.column_stack([data, qdot_cmd])
        phase1, phase2 = split_phases(data_with_vel, manual=False)
        vel1 = phase1[:, -1]             # commanded velocity deg/s
        vel2 = phase2[:, -1]
        vel_label = 'Commanded velocity (deg/s)'

    trq1 = phase1[:, IDX_ACTUAL_TRQ]
    trq2 = phase2[:, IDX_ACTUAL_TRQ]

    # Convert velocity to rev/s for FF calculation
    vel1_revs = vel1 / 360.0
    vel2_revs = vel2 / 360.0

    cur1 = phase1[:, IDX_CURRENT]
    cur2 = phase2[:, IDX_CURRENT]

    print(f"\nPhase 1 (FF off): {len(phase1)} samples")
    print(f"Phase 2 (FF on):  {len(phase2)} samples")

    # ── bin by commanded velocity and compute mean torque ─────────────────────
    bins = np.linspace(-200, 200, 21)  # deg/s bins
    centers = 0.5 * (bins[:-1] + bins[1:])

    def bin_mean(vel, trq, bins):
        means, stds, counts = [], [], []
        for lo, hi in zip(bins[:-1], bins[1:]):
            mask = (vel >= lo) & (vel < hi)
            if mask.sum() > 2:
                means.append(np.mean(trq[mask]))
                stds.append(np.std(trq[mask]) / np.sqrt(mask.sum()))
                counts.append(mask.sum())
            else:
                means.append(np.nan)
                stds.append(np.nan)
                counts.append(0)
        return np.array(means), np.array(stds)

    m1, s1 = bin_mean(vel1, trq1, bins)
    m2, s2 = bin_mean(vel2, trq2, bins)

    expected_offset = BACKEMF_FF_GAIN * centers / 360.0  # Nm

    # ── plot ──────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Left: scatter plot
    ax = axes[0]
    ax.scatter(vel1, trq1, s=2, alpha=0.3, color='steelblue', label='Phase 1 FF off')
    ax.scatter(vel2, trq2, s=2, alpha=0.3, color='tomato',    label='Phase 2 FF on')
    ax.set_xlabel(vel_label)
    ax.set_ylabel('Actual torque (Nm)')
    ax.set_title('Torque vs Velocity — raw scatter')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Right: binned means — expected FF shown as vertical shift of Phase 1.
    # If FF works perfectly, Phase 2 should sit on the "Phase 1 + FF" line.
    ax = axes[1]
    valid = ~np.isnan(m1) & ~np.isnan(m2)
    expected_phase2 = m1[valid] + expected_offset[valid]   # where Phase 2 should be
    ax.errorbar(centers[valid], m1[valid], yerr=s1[valid],
                fmt='o-', color='steelblue', label='Phase 1 FF off (mean ± SE)')
    ax.errorbar(centers[valid], m2[valid], yerr=s2[valid],
                fmt='s-', color='tomato',    label='Phase 2 FF on  (mean ± SE)')
    ax.plot(centers[valid], expected_phase2, 'k--', linewidth=1.5,
            label=f'Phase 1 + expected FF ({BACKEMF_FF_GAIN} × vel_rev_s)\n→ where Phase 2 should be')
    ax.set_xlabel(vel_label)
    ax.set_ylabel('Mean actual torque (Nm)')
    ax.set_title('Torque vs Velocity — binned means')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Print numerical summary
    print("\n--- Binned torque difference (Phase2 - Phase1) vs expected FF ---")
    print(f"{'Vel (deg/s)':>12} {'ΔT_meas (Nm)':>14} {'ΔT_expected (Nm)':>18}")
    for c, d1, d2, exp in zip(centers[valid], m1[valid], m2[valid],
                               expected_offset[valid]):
        print(f"{c:>12.1f} {d2 - d1:>14.4f} {exp:>18.4f}")

    plt.tight_layout()
    plt.savefig('backemf_validation.png', dpi=150)
    print("\nPlot saved to backemf_validation.png")

    # ── lag hypothesis plots (Phase 2 only) ───────────────────────────────────
    vel_meas_p2 = phase2[:, IDX_VELOCITY]          # deg/s, 120 Hz filtered
    t2          = phase2[:, IDX_TIMESTAMP]
    t2_norm     = t2 - t2[0]

    # Commanded acceleration — used to label each sample as accelerating/decelerating
    acc_cmd_p2  = np.gradient(vel2, t2)
    accel_mask  = acc_cmd_p2 > 0
    decel_mask  = acc_cmd_p2 <= 0

    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    fig2.suptitle('Filter Lag Hypothesis — Phase 2 (FF on)')

    # Left: commanded vs measured velocity over time
    # A visible phase shift here directly confirms filter lag.
    ax = axes2[0]
    ax.plot(t2_norm, vel2,        'k--', lw=1.2, label='Commanded velocity')
    ax.plot(t2_norm, vel_meas_p2, color='tomato', lw=1.2,
            label='Measured velocity (120 Hz filtered)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (deg/s)')
    ax.set_title('Commanded vs. measured velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Right: hysteresis check — torque vs commanded velocity, split by acceleration sign.
    # Lag causes the feedforward to under-compensate while speeding up and
    # over-compensate while slowing down, creating a loop.
    # Gain error produces a single offset line with no loop.
    ax = axes2[1]
    ax.scatter(vel2[accel_mask], trq2[accel_mask], s=2, alpha=0.4,
               color='steelblue', label='Accelerating')
    ax.scatter(vel2[decel_mask], trq2[decel_mask], s=2, alpha=0.4,
               color='tomato',    label='Decelerating')
    ax.axhline(0, color='k', lw=0.8, ls='--')
    ax.set_xlabel(vel_label)
    ax.set_ylabel('Actual torque (Nm)')
    ax.set_title('Hysteresis check (FF on)\nLoop → lag,  Parallel lines → gain error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('backemf_lag_test.png', dpi=150)
    print("Lag test plot saved to backemf_lag_test.png")

    # ── current vs velocity ───────────────────────────────────────────────────
    # If FF is reaching the current loop, Phase 2 slope should be much flatter.
    # Back-EMF drag creates current opposing motion: slope = -Ke/R = -1.3 A/(rev/s)
    # FF injects current in direction of motion: +BACKEMF_FF_GAIN per rev/s
    # So Phase 2 should sit above Phase 1 by FF_gain × velocity.
    KE_OVER_R = 0.75 / 0.577   # theoretical back-EMF drag slope A/(rev/s)

    fig3, axes3 = plt.subplots(1, 2, figsize=(14, 5))
    fig3.suptitle('Current vs Velocity — does FF actually reach the current loop?')

    # Left: raw scatter
    ax = axes3[0]
    ax.scatter(vel1_revs, cur1, s=2, alpha=0.3, color='steelblue', label='Phase 1 FF off')
    ax.scatter(vel2_revs, cur2, s=2, alpha=0.3, color='tomato',    label='Phase 2 FF on')
    vel_line = np.linspace(-0.3, 0.3, 100)
    ax.plot(vel_line, -KE_OVER_R * vel_line,             'k--',  lw=1.5, label=f'Expected FF off  slope=-{KE_OVER_R:.2f}')
    ax.plot(vel_line, -(KE_OVER_R - BACKEMF_FF_GAIN) * vel_line, 'k:',  lw=1.5, label=f'Expected FF on   slope=-{(KE_OVER_R-BACKEMF_FF_GAIN):.2f}')
    ax.set_xlabel('Measured velocity (rev/s)')
    ax.set_ylabel('Quadrature current (A)')
    ax.set_title('Raw scatter')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Right: binned means — easier to see the slope change
    cur_bins = np.linspace(-0.3, 0.3, 13)
    cur_centers = 0.5 * (cur_bins[:-1] + cur_bins[1:])

    def bin_mean_cur(vel, cur, bins):
        means, stds = [], []
        for lo, hi in zip(bins[:-1], bins[1:]):
            mask = (vel >= lo) & (vel < hi)
            if mask.sum() > 2:
                means.append(np.mean(cur[mask]))
                stds.append(np.std(cur[mask]) / np.sqrt(mask.sum()))
            else:
                means.append(np.nan)
                stds.append(np.nan)
        return np.array(means), np.array(stds)

    mc1, sc1 = bin_mean_cur(vel1_revs, cur1, cur_bins)
    mc2, sc2 = bin_mean_cur(vel2_revs, cur2, cur_bins)

    ax = axes3[1]
    valid = ~np.isnan(mc1) & ~np.isnan(mc2)
    expected_cur_phase2 = mc1[valid] + BACKEMF_FF_GAIN * cur_centers[valid]  # Phase 1 + FF shift
    ax.errorbar(cur_centers[valid], mc1[valid], yerr=sc1[valid],
                fmt='o-', color='steelblue', label='Phase 1 FF off')
    ax.errorbar(cur_centers[valid], mc2[valid], yerr=sc2[valid],
                fmt='s-', color='tomato',    label='Phase 2 FF on')
    ax.plot(cur_centers[valid], expected_cur_phase2, 'k--', lw=1.5,
            label=f'Phase 1 + expected FF ({BACKEMF_FF_GAIN} A/(rev/s))\n→ where Phase 2 should be')
    ax.set_xlabel('Measured velocity (rev/s)')
    ax.set_ylabel('Mean quadrature current (A)')
    ax.set_title('Binned means — compare slopes')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('backemf_current_debug.png', dpi=150)
    print("Current debug plot saved to backemf_current_debug.png")

    plt.show()


if __name__ == '__main__':
    main()
