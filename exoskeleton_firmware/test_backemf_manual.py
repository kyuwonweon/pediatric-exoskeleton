"""
Manual back-EMF feedforward test.

No trajectory is commanded. The user rotates the handle freely during
each phase. The reporter streams velocity and current for analysis.

  Phase 1 — FF OFF  (feel the natural electromagnetic drag)
  Phase 2 — FF ON   (feel with feedforward compensation)

Usage: run from the MicroPython REPL after main.py has initialised robot.
"""

import pyb

_T_PHASE  = 20.0   # s — duration of each phase (rotate back and forth during this)
_T_PAUSE  =  3.0   # s — rest between phases

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

robot.set_KD_parameter(0.0, 0.0)

print("=== Manual Back-EMF FF test ===")
print("Rotate the handle back and forth continuously during each phase.")
print(f"Phase duration: {_T_PHASE} s each")
print()

# Phase 1 — FF off
robot.set_backemf_feedforward(False)
print(">>> Phase 1 — FF OFF — start rotating now")
pyb.delay(int(_T_PHASE * 1000))
print(">>> Phase 1 done — hold still")
pyb.delay(int(_T_PAUSE * 1000))

# Phase 2 — FF on
robot.set_backemf_feedforward(True)
print(">>> Phase 2 — FF ON — start rotating now")
pyb.delay(int(_T_PHASE * 1000))
print(">>> Phase 2 done")

print("=== Test complete — stop bag recording ===")
