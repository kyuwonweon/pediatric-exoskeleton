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
    "reporter":      {"dim_robot_msg": 10},
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

    def test_no_servo_write_during_feedforward_compute(self):
        """_compute_backemf_feedforward must NOT write to the servo — only store the value."""
        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = 5.0
        self.robot.servo.reset_mock()

        self.robot._compute_backemf_feedforward()

        self.robot.servo.set_tau_offset.assert_not_called()


# ─────────────────────────────────────────────────────────────────────────────
class TestNoOverwrite(unittest.TestCase):
    """
    The original concern: TIMER_ROBOT (200 Hz) and TIMER_CONTROLLER (50 Hz) both
    wrote to TORQUE_LOOP_INPUT_OFFSET, stomping each other.

    Fix: _compute_backemf_feedforward() only updates _tau_backemf_ff.
         set_tau_offset(tau) always sends tau + _tau_backemf_ff.
    So both are present in every servo write, regardless of which timer fired.
    """

    R = 0.185

    def setUp(self):
        self.robot = _make_robot(motor_resistance=self.R)

    def test_set_tau_offset_merges_both(self):
        """Controller tau and feedforward tau both appear in the servo write."""
        tau_ctrl = 1.5   # Nm from controller
        velocity  = 4.0  # rev/s

        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = velocity

        # robot timer tick
        self.robot._compute_backemf_feedforward()
        tau_ff = self.robot._tau_backemf_ff

        # controller timer tick
        self.robot.set_tau_offset(tau_ctrl)

        self.robot.servo.set_tau_offset.assert_called_with(tau_ctrl + tau_ff)

    def test_feedforward_off_passes_only_ctrl_tau(self):
        """With feedforward disabled the servo receives exactly tau_ctrl."""
        tau_ctrl = 2.0
        self.robot._compute_backemf_feedforward()   # disabled → _tau_backemf_ff = 0
        self.robot.set_tau_offset(tau_ctrl)
        self.robot.servo.set_tau_offset.assert_called_with(tau_ctrl)

    def test_timer_sequence_does_not_clobber(self):
        """
        Simulate 4 robot ticks then 1 controller tick (the real 200/50 Hz ratio).
        After the controller tick the servo write must contain both tau values.
        """
        tau_ctrl = 0.8
        velocity  = 6.0

        self.robot.set_backemf_feedforward(True)
        self.robot.servo.velocity = velocity

        # 4 robot timer ticks (updating _tau_backemf_ff, no servo write)
        for _ in range(4):
            self.robot._compute_backemf_feedforward()

        tau_ff = self.robot._tau_backemf_ff   # should be BACKEMF_FF_GAIN * 6.0

        # 1 controller timer tick
        self.robot.set_tau_offset(tau_ctrl)

        # servo was written exactly once, with both contributions
        self.robot.servo.set_tau_offset.assert_called_once_with(tau_ctrl + tau_ff)

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


if __name__ == '__main__':
    unittest.main(verbosity=2)
