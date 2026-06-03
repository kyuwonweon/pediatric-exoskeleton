"""
Hardware back-EMF feedforward test.

Runs the same sinusoidal PVT sweep twice on the real motor:
  Phase 1 — FF OFF  (data[6] = 0)
  Phase 2 — FF ON   (data[6] = BACKEMF_FF_GAIN * velocity)

PVT mode requires continuous waypoint feeding at ~50 Hz with small dt steps,
exactly as ctrlOscillatorPos does it. A single large-dt waypoint won't move
the motor because the buffer empties before the next point arrives.

"""

import pyb
import math

# ── trajectory parameters ─────────────────────────────────────────────────────
_AMP       = 45.0    # deg — sinusoidal amplitude (keep well within ±90 deg limit)
_FREQ_HZ   = 0.25    # Hz  — oscillation frequency (one cycle = 4 s)
_N_CYCLES  = 2       # cycles per phase
_DT        = 0.02    # s   — PVT step size (50 Hz, matches controller rate)
_T_PAUSE   = 2.0     # s   — rest between phases so velocity reaches zero

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
def run_sweep():
    """
    Feed a sinusoidal trajectory via PVT at 50 Hz for N_CYCLES cycles.
    Each call to set_transition_point covers exactly _DT seconds of motion,
    matching the pattern used by ctrlOscillatorPos.
    """
    n_steps = int((_N_CYCLES / _FREQ_HZ) / _DT)
    for i in range(n_steps):
        t     = i * _DT
        q     = _AMP * math.sin(2 * math.pi * _FREQ_HZ * t)
        qdot  = _AMP * 2 * math.pi * _FREQ_HZ * math.cos(2 * math.pi * _FREQ_HZ * t)
        robot._q_des_test = q
        robot.set_transition_point(q, qdot, _DT)
        pyb.delay(15)   # 15 ms delay + ~5 ms loop overhead ≈ 20 ms total


def run_phase(label, ff_enabled):
    robot.set_backemf_feedforward(ff_enabled)
    # return to zero before starting so both phases begin from the same position
    robot.set_transition_point(0.0, 0.0, _DT)
    pyb.delay(int(_T_PAUSE * 1000))
    print(">>> " + label)
    run_sweep()
    # come to rest at zero
    robot.set_transition_point(0.0, 0.0, _DT)
    pyb.delay(int(_T_PAUSE * 1000))


# ── test sequence ─────────────────────────────────────────────────────────────
_duration = _N_CYCLES / _FREQ_HZ
_peak_vel  = _AMP * 2 * math.pi * _FREQ_HZ   # deg/s at zero crossing

print("=== Back-EMF FF hardware test ===")
print("Trajectory: sine  amp =", _AMP, "deg  freq =", _FREQ_HZ, "Hz")
print("Peak velocity :", round(_peak_vel, 1), "deg/s =",
      round(_peak_vel / 360, 3), "rev/s")
print("Phase duration:", _duration, "s  (", _N_CYCLES, "cycles )")
print("Expected peak FF:", round(0.169 * _peak_vel / 360, 4), "Nm")
print()

run_phase("Phase 1 — FF OFF", ff_enabled=False)
run_phase("Phase 2 — FF ON",  ff_enabled=True)

print("=== Test complete — stop bag recording ===")
