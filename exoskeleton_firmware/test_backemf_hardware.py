"""
Hardware back-EMF + acceleration feedforward test.

Runs the same sinusoidal PVT sweep three times:
  Phase 1 — FF OFF
  Phase 2 — Back-EMF FF ON  (ACC_FF_GAIN = 0)
  Phase 3 — Back-EMF FF + Acc FF ON

Phase marker (1.0 / 2.0 / 3.0) is written to _q_des_test during the pause
between sweeps. During the sweep _q_des_test carries the commanded position
so analyze_backemf.py can plot position tracking error.

PVT mode requires continuous waypoint feeding at ~50 Hz with small dt steps,
exactly as ctrlOscillatorPos does it. A single large-dt waypoint won't move
the motor because the buffer empties before the next point arrives.
"""

import pyb
import math

# ── trajectory parameters ─────────────────────────────────────────────────────
_AMP       = 30.0    # deg — sinusoidal amplitude  (peak vel ≈ 47 deg/s)
_FREQ_HZ   = 0.25    # Hz  — oscillation frequency (one cycle = 4 s)
_N_CYCLES  = 4       # cycles per phase — more cycles = more samples for averaging
_DT        = 0.02    # s   — PVT step size (50 Hz, matches controller rate)
_T_PAUSE   = 2.0     # s   — rest between phases so velocity reaches zero
_ACC_FF_GAIN = 0.1   # tune this — same as test_backemf_manual.py

# Clip amplitude to 90 % of the tighter position limit so the sine never hits
# the hardware clamp in set_transition_point (which would corrupt the velocity).
_POS_LIMIT = min(abs(robot.position_limits[0]), abs(robot.position_limits[1]))
_AMP       = min(_AMP, 0.9 * _POS_LIMIT)

try:
    robot       # already initialised by main.py
    print("Reusing existing robot instance")
except NameError:
    # Fallback
    from lib import config
    from lib.ll_common import mode_switch
    from lib.IO.NetworkManager import NetworkManager
    from lib.Robots.robotCMBenchTop import CMBenchTop
    from lib.IO.ReporterWifiUDP import ReporterWifiUDP
    from lib.CtrlFactory.CtrlFactory import CtrlFactory

    _PARAM_FILE  = "config_params/params.json"
    _REPORTER_IP = "192.168.0.252"

    selector          = mode_switch.selector
    device_parameters = config.DeviceParameters(selector.device_id)
    robot    = CMBenchTop(_PARAM_FILE, "CubeMars_BenchTop", device_parameters)
    ctrlFact = CtrlFactory(robot, _PARAM_FILE)

    netManager = NetworkManager()
    IP = netManager.connect_WLAN()
    print("IP:", IP)

    reporter = ReporterWifiUDP(ctrlFact, _PARAM_FILE, _REPORTER_IP)
    reporter.enable_reporter()

    robot.enable(True)
    pyb.delay(500)

# Ensure PVT mode is active
robot.change_to_PVT_profiler()
pyb.delay(200)


# ── helpers ───────────────────────────────────────────────────────────────────
def run_sweep(phase_marker):
    """
    Feed a sinusoidal trajectory via PVT at 50 Hz for N_CYCLES cycles.
    _q_des_test carries commanded position during sweep for tracking error plots.
    """
    n_steps = int((_N_CYCLES / _FREQ_HZ) / _DT)
    for i in range(n_steps):
        t     = i * _DT
        q     = _AMP * math.sin(2 * math.pi * _FREQ_HZ * t)
        qdot  = _AMP * 2 * math.pi * _FREQ_HZ * math.cos(2 * math.pi * _FREQ_HZ * t)
        robot._q_des_test = q
        robot.set_transition_point(q, qdot, _DT)
        pyb.delay(15)   # 15 ms delay + ~5 ms loop overhead ≈ 20 ms total


def run_phase(label, phase_marker, ff_enabled, acc_ff_gain):
    # set marker and FF state during pause (while motor is still)
    robot._q_des_test = phase_marker
    robot.ACC_FF_GAIN = acc_ff_gain
    robot.set_backemf_feedforward(ff_enabled)
    # return to zero before starting
    robot.set_transition_point(0.0, 0.0, _DT)
    pyb.delay(int(_T_PAUSE * 1000))
    print(">>> " + label)
    run_sweep(phase_marker)
    # come to rest
    robot._q_des_test = phase_marker
    robot.set_transition_point(0.0, 0.0, _DT)
    pyb.delay(int(_T_PAUSE * 1000))


# ── test sequence ─────────────────────────────────────────────────────────────
_duration = _N_CYCLES / _FREQ_HZ
_peak_vel  = _AMP * 2 * math.pi * _FREQ_HZ

print("=== Back-EMF + Acc FF hardware test ===")
print("Trajectory: sine  amp =", _AMP, "deg  freq =", _FREQ_HZ, "Hz")
print("Peak velocity:", round(_peak_vel, 1), "deg/s")
print("Phase duration:", _duration, "s  (", _N_CYCLES, "cycles)")
print("ACC_FF_GAIN =", _ACC_FF_GAIN)
print()

run_phase("Phase 1 — FF OFF",              1.0, ff_enabled=False, acc_ff_gain=0.0)
run_phase("Phase 2 — Back-EMF FF ON",      2.0, ff_enabled=True,  acc_ff_gain=0.0)
run_phase("Phase 3 — Back-EMF + Acc FF",   3.0, ff_enabled=True,  acc_ff_gain=_ACC_FF_GAIN)

# restore safe defaults
robot.ACC_FF_GAIN = 0.0
robot.set_backemf_feedforward(False)

print("=== Test complete — stop bag recording ===")
