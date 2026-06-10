"""
Manual back-EMF + acceleration feedforward test.

No trajectory is commanded. The user rotates the handle freely during
each phase. The reporter streams velocity, acceleration, and current for analysis.

  Phase 1 — FF OFF        (feel the natural electromagnetic drag)
  Phase 2 — Back-EMF FF ON  (velocity compensation only, ACC_FF_GAIN=0)
  Phase 3 — Back-EMF + Acc FF ON  (adds inertia compensation at direction changes)

Usage: run from the MicroPython REPL after main.py has initialised robot.
"""

import pyb

_T_PHASE      = 20.0   # s — duration of each phase
_T_PAUSE      = 3.0    # s — rest between phases
_ACC_FF_GAIN  = 0.1   # tune this — theoretical is J_motor*n/Kt, start small

try:
    robot
    print("Reusing existing robot instance")
except NameError:
    from lib import config
    from lib.ll_common import mode_switch
    from lib.IO.NetworkManager import NetworkManager
    from lib.Robots.robotCMBenchTop import CMBenchTop
    from lib.IO.ReporterWifiUDP import ReporterWifiUDP
    from lib.CtrlFactory.CtrlFactory import CtrlFactory

    _PARAM_FILE  = "config_params/params.json"
    _REPORTER_IP = "192.168.0.252"

    selector = mode_switch.selector
    device_parameters = config.DeviceParameters(selector.device_id)
    robot = CMBenchTop(_PARAM_FILE, "CubeMars_BenchTop", device_parameters)
    ctrlFact = CtrlFactory(robot, _PARAM_FILE)

    netManager = NetworkManager()
    IP = netManager.connect_WLAN()
    print("IP:", IP)

    reporter = ReporterWifiUDP(ctrlFact, _PARAM_FILE, _REPORTER_IP)
    reporter.enable_reporter()

    robot.enable(True)
    pyb.delay(500)

robot.set_KD_parameter(0.0, 0.0)

print("=== Manual Back-EMF + Acceleration FF test ===")
print("Rotate the handle back and forth continuously during each phase.")
print("Focus on direction reversals — that is when acc FF activates.")
print(f"Phase duration: {_T_PHASE} s each  |  ACC_FF_GAIN = {_ACC_FF_GAIN}")
print()

# Phase 1 — all FF off
robot._q_des_test = 1.0
robot.ACC_FF_GAIN = 0.0
robot.set_backemf_feedforward(False)
print(">>> Phase 1 — FF OFF — start rotating now")
pyb.delay(int(_T_PHASE * 1000))
print(">>> Phase 1 done — hold still")
pyb.delay(int(_T_PAUSE * 1000))

# Phase 2 — back-EMF FF only (acc gain stays 0)
robot._q_des_test = 2.0
robot.ACC_FF_GAIN = 0.0
robot.set_backemf_feedforward(True)
print(">>> Phase 2 — Back-EMF FF ON, Acc FF OFF — start rotating now")
pyb.delay(int(_T_PHASE * 1000))
print(">>> Phase 2 done — hold still")
pyb.delay(int(_T_PAUSE * 1000))

# Phase 3 — back-EMF FF + acceleration FF
robot._q_des_test = 3.0
robot.ACC_FF_GAIN = _ACC_FF_GAIN
robot.set_backemf_feedforward(True)
print(">>> Phase 3 — Back-EMF FF + Acc FF ON — focus on reversals")
pyb.delay(int(_T_PHASE * 1000))
print(">>> Phase 3 done")

# restore safe default
robot.ACC_FF_GAIN = 0.0
robot.set_backemf_feedforward(False)

print("=== Test complete — stop bag recording ===")
