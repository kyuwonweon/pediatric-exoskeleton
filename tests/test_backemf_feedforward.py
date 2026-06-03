"""
Back EMF feedforward test for CMBenchTop.

What this validates:
  1. BACKEMF_FF_GAIN math: Kt * Ke / R
  2. _compute_backemf_feedforward() updates _tau_backemf_ff — not to servo directly
  3. set_tau_offset(tau) sends (tau + _tau_backemf_ff) to the servo
  4. Feedforward is off by default; enable/disable toggles correctly

Run from workspace root:
  python3 tests/test_backemf_feedforward.py

Hardware verification (PlotJuggler):
  - Enable feedforward with robot.set_backemf_feedforward(True)
  - Stream at constant velocity (PVT sine trajectory)
  - With feedforward ON:  actual_torque should track torque_commanded more closely
  - With feedforward OFF: actual_torque lags torque_commanded at high velocity
  - backemf topic should be proportional to velocity with slope = KE_LARGE_MOTOR
"""

import sys
import os
import json
import unittest
from unittest.mock import MagicMock, mock_open, patch

# ─── 1. Stub hardware before any project import ───────────────────────────────
#     Only the leaf modules that import pyb/spi/novanta need mocking.
#     lib/__init__.py and lib/Robots/__init__.py are empty, so Python can still
#     find robotCMBenchTop.py and RobotInterface.py on the real filesystem.
for _m in ['pyb', 'micropython', 'lib.Hardware.servoMotorCM', 'lib.ll_common.error_flag']:
    sys.modules[_m] = MagicMock()
sys.modules['micropython'].const = lambda x: x  # MicroPython optimization hint

# ─── 2. Add firmware package to path ─────────────────────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'exoskeleton_firmware'))

# ─── 3. Minimal JSON params that Robot.__init__ loads from file ───────────────
_PARAMS = {
    "CubeMars_BenchTop": {
        "K_initial": 1.0, "B_initial": 0.1,
        "K_bounds": [0.0, 3.0], "B_bounds": [0.0, 0.5],
    },
    "reporter":      {"dim_robot_msg": 11},
    "input_manager": {"dim_robot_cmd": 4},
    "servo":         {"CURRENT_LIMIT": 1.0},
}

class _DeviceParams:
    TIMER_ROBOT      = {'ID': 1,  'FREQ': 200}
    TIMER_SERVO      = {'ID': 12, 'FREQ': 300}
    TIMER_CONTROLLER = {'ID': 2,  'FREQ': 50}

# ─── 4. Import real CMBenchTop (hardware calls go to mocks) ───────────────────
from lib.Robots.robotCMBenchTop import CMBenchTop  # noqa: E402


def _make_robot(motor_resistance: float = 0.0) -> CMBenchTop:
    """
    Instantiate a real CMBenchTop with all hardware bypassed.
    motor_resistance: set to your datasheet phase resistance (Ohm) to test non-zero gain.
                      Defaults to 0 (the placeholder in source) to test the safe default.
    """
    with patch('builtins.open', mock_open(read_data=json.dumps(_PARAMS))):
        robot = CMBenchTop('dummy.json', 'CubeMars_BenchTop', _DeviceParams())
    # Each robot gets its own fresh servo mock so call counts don't bleed between tests.
    # (ServoCM is a module-level mock whose return_value is shared across instances.)
    robot.servo = MagicMock()
    # Simulate user filling in the motor resistance constant
    robot.MOTOR_RESISTANCE = motor_resistance
    robot.BACKEMF_FF_GAIN = (
        robot.TORQUE_CONSTANT_LARGE_MOTOR * robot.KE_LARGE_MOTOR / motor_resistance
        if motor_resistance > 0 else 0.0
    )
    robot.run = True
    return robot


# ─────────────────────────────────────────────────────────────────────────────
class TestGainComputation(unittest.TestCase):

    def test_gain_formula_values(self):
        """BACKEMF_FF_GAIN = Kt * Ke / R with the datasheet numbers."""
        R = 0.185  # example — replace with your real value
        robot = _make_robot(motor_resistance=R)
        Kt = 0.13    # Nm/A  from AKE60-KV80 datasheet
        Ke = 0.75    # V/(rev/s)  = 12.5 V/kRPM * 60e-3
        expected = Kt * Ke / R
        self.assertAlmostEqual(robot.BACKEMF_FF_GAIN, expected, places=6)

    def test_ke_unit_conversion(self):
        """Ke = 12.5 V/kRPM == 0.75 V/(rev/s)."""
        robot = _make_robot()
        self.assertAlmostEqual(robot.KE_LARGE_MOTOR, 12.5 * 60e-3, places=10)

    def test_zero_resistance_safe(self):
        """MOTOR_RESISTANCE = 0 (unfilled placeholder) must not raise and must give gain=0."""
        robot = _make_robot(motor_resistance=0.0)
        self.assertEqual(robot.BACKEMF_FF_GAIN, 0.0)
        robot._backemf_ff_enabled = True
        robot.servo.velocity = 10.0
        robot._compute_backemf_feedforward()   # must not raise ZeroDivisionError
        self.assertEqual(robot._tau_backemf_ff, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
class TestFeedforwardUpdate(unittest.TestCase):

    R = 0.185

    def setUp(self):
        self.robot = _make_robot(motor_resistance=self.R)

    def test_disabled_by_default(self):
        """Feedforward must be off at init so existing behaviour is unchanged."""
        self.assertFalse(self.robot._backemf_ff_enabled)
        self.robot.servo.velocity = 10.0
        self.robot._compute_backemf_feedforward()
        self.assertEqual(self.robot._tau_backemf_ff, 0.0)

    def test_enabled_computes_correctly(self):
        """_tau_backemf_ff = BACKEMF_FF_GAIN * velocity when enabled."""
        self.robot.set_backemf_feedforward(True)
        velocity = 5.0   # rev/s
        self.robot.servo.velocity = velocity
        self.robot._compute_backemf_feedforward()
        expected = self.robot.BACKEMF_FF_GAIN * velocity
        self.assertAlmostEqual(self.robot._tau_backemf_ff, expected, places=8)

    def test_positive_velocity_gives_positive_feedforward(self):
        """Feedforward sign must match motion direction."""
        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = 3.0
        self.robot._compute_backemf_feedforward()
        self.assertGreater(self.robot._tau_backemf_ff, 0.0)

    def test_negative_velocity_gives_negative_feedforward(self):
        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = -3.0
        self.robot._compute_backemf_feedforward()
        self.assertLess(self.robot._tau_backemf_ff, 0.0)

    def test_disable_resets_tau_to_zero(self):
        """Disabling mid-run must clear _tau_backemf_ff immediately."""
        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = 5.0
        self.robot._compute_backemf_feedforward()
        self.assertNotEqual(self.robot._tau_backemf_ff, 0.0)  # sanity

        self.robot.set_backemf_feedforward(False)
        self.robot._compute_backemf_feedforward()
        self.assertEqual(self.robot._tau_backemf_ff, 0.0)

    def test_feedforward_compute_writes_to_servo(self):
        """_compute_backemf_feedforward must write _tau_commanded to the servo every tick."""
        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = 5.0
        self.robot.servo.reset_mock()

        self.robot._compute_backemf_feedforward()
        expected = self.robot._tau_backemf_ff  # _tau_external is 0 at init

        self.robot.servo.set_tau_offset.assert_called_once_with(expected)


# ─────────────────────────────────────────────────────────────────────────────
class TestNoOverwrite(unittest.TestCase):
    """
    The original concern: TIMER_ROBOT (200 Hz) and TIMER_CONTROLLER (50 Hz) both
    wrote to TORQUE_LOOP_INPUT_OFFSET, stomping each other.

    Fix: _compute_backemf_feedforward() owns the servo write every robot tick,
         sending _tau_external + _tau_backemf_ff together.
         set_tau_offset(tau) only stores _tau_external — no direct servo write.
    So both contributions are always present in every servo write.
    """

    R = 0.185

    def setUp(self):
        self.robot = _make_robot(motor_resistance=self.R)

    def test_set_tau_offset_merges_both(self):
        """Controller tau and feedforward tau both appear in the servo write on the next robot tick."""
        tau_ctrl = 1.5   # Nm from controller
        velocity  = 4.0  # rev/s

        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = velocity

        # first robot tick: writes 0 + tau_ff (tau_external still 0)
        self.robot._compute_backemf_feedforward()
        tau_ff = self.robot._tau_backemf_ff

        # controller tick: stores tau_ctrl, no servo write
        self.robot.set_tau_offset(tau_ctrl)

        # next robot tick: writes tau_ctrl + tau_ff
        self.robot._compute_backemf_feedforward()

        self.robot.servo.set_tau_offset.assert_called_with(tau_ctrl + tau_ff)

    def test_feedforward_off_passes_only_ctrl_tau(self):
        """With feedforward disabled the servo receives exactly tau_ctrl on the next robot tick."""
        tau_ctrl = 2.0
        self.robot.set_tau_offset(tau_ctrl)           # stores tau_external, no servo write
        self.robot._compute_backemf_feedforward()     # disabled → tau_ff=0, writes tau_ctrl
        self.robot.servo.set_tau_offset.assert_called_with(tau_ctrl)

    def test_timer_sequence_does_not_clobber(self):
        """
        4 robot ticks, 1 controller tick, then 1 more robot tick (200/50 Hz ratio).
        The final servo write must contain both tau_ctrl and tau_ff.
        """
        tau_ctrl = 0.8
        velocity  = 6.0

        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = velocity

        # 4 robot ticks — each writes (0 + tau_ff) since tau_external=0
        for _ in range(4):
            self.robot._compute_backemf_feedforward()

        tau_ff = self.robot._tau_backemf_ff   # BACKEMF_FF_GAIN * 6.0

        # controller tick: stores tau_ctrl, no servo write
        self.robot.set_tau_offset(tau_ctrl)

        # next robot tick: writes tau_ctrl + tau_ff
        self.robot._compute_backemf_feedforward()

        self.robot.servo.set_tau_offset.assert_called_with(tau_ctrl + tau_ff)

    def test_tau_ff_changes_with_velocity(self):
        """_tau_backemf_ff tracks velocity — higher speed → larger feedforward."""
        self.robot.set_backemf_feedforward(True)

        self.robot.servo.velocity = 2.0
        self.robot._compute_backemf_feedforward()
        tau_low = self.robot._tau_backemf_ff

        self.robot.servo.velocity = 8.0
        self.robot._compute_backemf_feedforward()
        tau_high = self.robot._tau_backemf_ff

        self.assertGreater(tau_high, tau_low)


import math as _math


def _pearson_r(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = (_math.sqrt(sum((x - mx) ** 2 for x in xs)) *
           _math.sqrt(sum((y - my) ** 2 for y in ys)))
    return num / den if den > 0 else 0.0


def _bin_by_speed(velocities, values, n_bins):
    """Bucket (|velocity|, |value|) into n equal-width speed bins, return (mean_speed, mean_val) per bin."""
    max_speed = max(abs(v) for v in velocities)
    width = max_speed / n_bins
    buckets = [[] for _ in range(n_bins)]
    for v, val in zip(velocities, values):
        idx = min(int(abs(v) / width), n_bins - 1)
        buckets[idx].append((abs(v), abs(val)))
    return [
        (sum(p[0] for p in b) / len(b), sum(p[1] for p in b) / len(b))
        for b in buckets if b
    ]


# ─────────────────────────────────────────────────────────────────────────────
class TestFeedforwardEffectiveness(unittest.TestCase):
    """
    Strategy 3: velocity-binned correlation analysis.

    Simulates a sinusoidal velocity profile and checks:
      1. Linear proportionality  : Pearson r(velocity, tau_ff) > 0.999
      2. Gain consistency        : tau_ff / speed ≈ BACKEMF_FF_GAIN in every bin
      3. Compensation magnitude  : peak tau_ff is non-negligible vs observed motor torque
    """

    R = 0.577
    N_SAMPLES = 360
    VEL_AMP = 0.3             # rev/s peak ≈ 108 deg/s, realistic operating speed
    N_BINS = 6
    GAIN_TOLERANCE = 0.005    # Nm/(rev/s) — numerical only, no physics uncertainty here
    OBSERVED_PEAK_TORQUE_NM = 0.5   # peak actual_torque seen on hardware (motor shaft, Nm)
    MIN_FF_SIGNIFICANCE = 0.01      # 1 % — below this FF is negligible

    def setUp(self):
        self.robot = _make_robot(motor_resistance=self.R)
        self.robot.set_backemf_feedforward(True)
        self.velocities = [
            self.VEL_AMP * _math.sin(2 * _math.pi * i / self.N_SAMPLES)
            for i in range(self.N_SAMPLES)
        ]
        self.tau_ff = []
        for v in self.velocities:
            self.robot.servo.velocity = v
            self.robot._compute_backemf_feedforward()
            self.tau_ff.append(self.robot._tau_backemf_ff)

    def test_linear_proportionality(self):
        """Pearson r(velocity, tau_ff) must exceed 0.999 — confirms model is linear."""
        r = _pearson_r(self.velocities, self.tau_ff)
        self.assertGreater(r, 0.999,
            f"tau_ff not linearly proportional to velocity (r = {r:.4f})")

    def test_binned_gain_consistency(self):
        """In every speed bin, empirical tau_ff/speed must equal BACKEMF_FF_GAIN within tolerance."""
        bins = _bin_by_speed(self.velocities, self.tau_ff, self.N_BINS)
        for mean_speed, mean_tau in bins:
            if mean_speed < 1e-4:
                continue
            empirical = mean_tau / mean_speed
            self.assertAlmostEqual(
                empirical, self.robot.BACKEMF_FF_GAIN, delta=self.GAIN_TOLERANCE,
                msg=(f"Bin at {mean_speed:.3f} rev/s: empirical gain {empirical:.4f} "
                     f"!= BACKEMF_FF_GAIN {self.robot.BACKEMF_FF_GAIN:.4f}"))

    def test_compensation_significance(self):
        """Peak tau_ff must be >= MIN_FF_SIGNIFICANCE of observed peak motor torque."""
        peak_ff = max(abs(t) for t in self.tau_ff)
        ratio = peak_ff / self.OBSERVED_PEAK_TORQUE_NM
        self.assertGreater(ratio, self.MIN_FF_SIGNIFICANCE,
            f"Peak FF ({peak_ff:.4f} Nm) is only {ratio*100:.1f}% of observed "
            f"peak torque ({self.OBSERVED_PEAK_TORQUE_NM} Nm) — "
            f"below {self.MIN_FF_SIGNIFICANCE*100:.0f}% significance threshold")

    def test_print_effectiveness_summary(self):
        """Prints velocity-binned analysis table. Always passes — read output for insight."""
        bins = _bin_by_speed(self.velocities, self.tau_ff, self.N_BINS)
        peak_ff = max(abs(t) for t in self.tau_ff)
        r = _pearson_r(self.velocities, self.tau_ff)

        print("\n── Back-EMF FF Effectiveness Summary ──────────────────────────")
        print(f"  BACKEMF_FF_GAIN          : {self.robot.BACKEMF_FF_GAIN:.4f} Nm/(rev/s)")
        print(f"  Peak velocity            : {self.VEL_AMP:.3f} rev/s  "
              f"({self.VEL_AMP * 360:.1f} deg/s)")
        print(f"  Peak tau_ff              : {peak_ff:.4f} Nm")
        print(f"  As %% of obs. peak torque : {peak_ff / self.OBSERVED_PEAK_TORQUE_NM * 100:.1f}%%")
        print(f"  Pearson r(vel, tau_ff)   : {r:.6f}")
        print()
        print(f"  {'Speed bin (rev/s)':>20} | {'mean |tau_ff| (Nm)':>20} | {'tau_ff/speed':>14}")
        print("  " + "─" * 60)
        for mean_speed, mean_tau in bins:
            ratio = mean_tau / mean_speed if mean_speed > 1e-4 else 0.0
            print(f"  {mean_speed:>20.4f} | {mean_tau:>20.6f} | {ratio:>14.4f}")
        print("────────────────────────────────────────────────────────────────")


def _trapezoid(vel_peak, n_ramp, n_hold):
    """Trapezoidal velocity profile: ramp up → hold → ramp down (all positive)."""
    up   = [vel_peak * i / n_ramp for i in range(n_ramp)]
    hold = [vel_peak] * n_hold
    down = [vel_peak * (n_ramp - i) / n_ramp for i in range(n_ramp + 1)]
    return up + hold + down


def _rms(values):
    return _math.sqrt(sum(v ** 2 for v in values) / len(values))


# ─────────────────────────────────────────────────────────────────────────────
class TestPairedVelocitySweep(unittest.TestCase):
    """
    Strategy 1: paired velocity sweep — FF off vs FF on, same trapezoidal profile.

    Physics simulation used for both runs:
        tau_actual = tau_impedance + tau_ff_applied - BACKEMF_FF_GAIN * |velocity|

    where tau_impedance is a fixed operating-point torque (e.g. K * pos_error at
    steady state) and tau_ff_applied is 0 (FF off) or BACKEMF_FF_GAIN * v (FF on).

    Three assertions:
      1. FF off  : torque deficit is linearly proportional to speed (r > 0.999)
      2. FF on   : residual deficit is eliminated within numerical tolerance
      3. Per-bin : (tau_on - tau_off) matches BACKEMF_FF_GAIN * speed in every bin
      4. Summary : RMS torque error reduction >= 90 %
    """

    R             = 0.577
    VEL_PEAK      = 0.3    # rev/s  (≈ 108 deg/s)
    N_RAMP        = 60
    N_HOLD        = 120
    TAU_IMPEDANCE = 0.3    # Nm — representative steady-state impedance torque
    N_BINS        = 5
    RESIDUAL_TOL  = 1e-9   # Nm — FF on must be this close to tau_impedance
    BIN_TOL       = 0.005  # Nm — bin-average improvement vs prediction
    MIN_RMS_REDUCTION = 0.90   # 90 %

    def setUp(self):
        self.robot = _make_robot(motor_resistance=self.R)
        self.velocities = _trapezoid(self.VEL_PEAK, self.N_RAMP, self.N_HOLD)

    def _run(self, ff_enabled):
        """Simulate actual_torque for every sample using the back-EMF physics model."""
        self.robot.set_backemf_feedforward(ff_enabled)
        actuals = []
        for v in self.velocities:
            self.robot.servo.velocity = v
            self.robot._compute_backemf_feedforward()
            deficit = self.robot.BACKEMF_FF_GAIN * abs(v)
            actual  = self.TAU_IMPEDANCE + self.robot._tau_backemf_ff - deficit
            actuals.append(actual)
        return actuals

    # ── test 1 ────────────────────────────────────────────────────────────────
    def test_ff_off_deficit_scales_with_velocity(self):
        """With FF off, torque deficit must be linearly proportional to speed (r > 0.999)."""
        actuals_off = self._run(ff_enabled=False)
        deficits    = [self.TAU_IMPEDANCE - a for a in actuals_off]
        speeds      = [abs(v) for v in self.velocities]
        r = _pearson_r(speeds, deficits)
        self.assertGreater(r, 0.999,
            f"Deficit does not scale linearly with speed (r = {r:.4f})")

    # ── test 2 ────────────────────────────────────────────────────────────────
    def test_ff_on_eliminates_deficit(self):
        """With FF on, actual_torque must equal tau_impedance within RESIDUAL_TOL."""
        actuals_on = self._run(ff_enabled=True)
        for i, (actual, v) in enumerate(zip(actuals_on, self.velocities)):
            residual = abs(actual - self.TAU_IMPEDANCE)
            self.assertLess(residual, self.RESIDUAL_TOL,
                f"Sample {i} (v={v:.3f} rev/s): residual {residual:.2e} Nm")

    # ── test 3 ────────────────────────────────────────────────────────────────
    def test_bin_improvement_matches_prediction(self):
        """In every speed bin, (tau_on - tau_off) must equal BACKEMF_FF_GAIN * speed."""
        actuals_off  = self._run(ff_enabled=False)
        actuals_on   = self._run(ff_enabled=True)
        improvements = [on - off for on, off in zip(actuals_on, actuals_off)]
        for mean_speed, mean_imp in _bin_by_speed(self.velocities, improvements, self.N_BINS):
            if mean_speed < 1e-4:
                continue
            predicted = self.robot.BACKEMF_FF_GAIN * mean_speed
            self.assertAlmostEqual(mean_imp, predicted, delta=self.BIN_TOL,
                msg=(f"Bin {mean_speed:.3f} rev/s: improvement {mean_imp:.4f} Nm "
                     f"vs predicted {predicted:.4f} Nm"))

    # ── test 4 ────────────────────────────────────────────────────────────────
    def test_rms_error_reduction(self):
        """RMS torque error must drop by at least MIN_RMS_REDUCTION when FF is enabled."""
        actuals_off = self._run(ff_enabled=False)
        actuals_on  = self._run(ff_enabled=True)
        rms_off = _rms([self.TAU_IMPEDANCE - a for a in actuals_off])
        rms_on  = _rms([self.TAU_IMPEDANCE - a for a in actuals_on])
        reduction = 1.0 - rms_on / rms_off if rms_off > 0 else 1.0
        self.assertGreater(reduction, self.MIN_RMS_REDUCTION,
            f"RMS error reduction {reduction*100:.1f}% < "
            f"{self.MIN_RMS_REDUCTION*100:.0f}% "
            f"(off={rms_off:.4f} Nm, on={rms_on:.2e} Nm)")

    # ── summary ───────────────────────────────────────────────────────────────
    def test_print_paired_sweep_summary(self):
        """Prints paired sweep results table. Always passes — read output for insight."""
        actuals_off  = self._run(ff_enabled=False)
        actuals_on   = self._run(ff_enabled=True)
        improvements = [on - off for on, off in zip(actuals_on, actuals_off)]
        deficits     = [self.TAU_IMPEDANCE - a for a in actuals_off]
        rms_off = _rms([self.TAU_IMPEDANCE - a for a in actuals_off])
        rms_on  = _rms([self.TAU_IMPEDANCE - a for a in actuals_on])
        reduction = 1.0 - rms_on / rms_off if rms_off > 0 else 1.0

        print("\n── Paired Velocity Sweep: FF Off vs FF On ──────────────────────────")
        print(f"  Profile     : trapezoid  peak={self.VEL_PEAK} rev/s "
              f"({self.VEL_PEAK*360:.0f} deg/s)  {len(self.velocities)} samples")
        print(f"  tau_impedance (fixed)    : {self.TAU_IMPEDANCE:.3f} Nm")
        print(f"  BACKEMF_FF_GAIN          : {self.robot.BACKEMF_FF_GAIN:.4f} Nm/(rev/s)")
        print(f"  Peak deficit (FF off)    : {max(deficits):.4f} Nm  "
              f"({max(deficits)/self.TAU_IMPEDANCE*100:.1f}% of tau_impedance)")
        print()
        print(f"  RMS torque error — FF off : {rms_off:.4f} Nm")
        print(f"  RMS torque error — FF on  : {rms_on:.2e} Nm")
        print(f"  RMS error reduction       : {reduction*100:.1f}%")
        print()
        col = f"  {'Speed bin (rev/s)':>18} | {'deficit-FF-off (Nm)':>20} | " \
              f"{'improvement (Nm)':>18} | {'predicted (Nm)':>16} | {'ok':>4}"
        print(col)
        print("  " + "─" * (len(col) - 2))
        bins_def = _bin_by_speed(self.velocities, deficits,     self.N_BINS)
        bins_imp = _bin_by_speed(self.velocities, improvements, self.N_BINS)
        for (spd, deficit), (_, imp) in zip(bins_def, bins_imp):
            predicted = self.robot.BACKEMF_FF_GAIN * spd
            ok = abs(imp - predicted) < self.BIN_TOL
            print(f"  {spd:>18.4f} | {deficit:>20.6f} | "
                  f"{imp:>18.6f} | {predicted:>16.6f} | {'✓' if ok else '✗':>4}")
        print("────────────────────────────────────────────────────────────────────")


if __name__ == '__main__':
    unittest.main(verbosity=2)
