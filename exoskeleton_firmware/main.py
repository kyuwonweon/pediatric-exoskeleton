import time

import micropython                                  # type: ignore
from micropython import const                       # type: ignore
import pyb                                          # type: ignore
import json
from lib import config
from lib.ll_common import mode_switch

from lib.IO.NetworkManager import NetworkManager, AccessPointManager
from lib.Robots.robotFoxKnee import FoxKnee
from lib.Robots.robotCMBenchTop import CMBenchTop
from lib.Controllers.ctrlOscillatorPos import ctrlOscillatorPos
from lib.Controllers.ctrlFBTransp_Swing import ctrl_FB_Transp
from lib.Controllers.ctrlFFTransp import ctrl_FF_Transp
from lib.Controllers.ctrlTelop import ctrl_Teleop
from lib.Controllers.ctrlSMWalking import CtrlSMWalking
from lib.IO.ReporterWifi import ReporterWifi
from lib.IO.ReporterWifiUDP import ReporterWifiUDP
from lib.IO.InManTCP import IMTCP
from lib.IO.InManUDP import IMUDP
from lib.CtrlFactory.CtrlFactory import CtrlFactory

# from lib.IO.ReporterStdOut import ReporterStdOut

_PARAM_FILE = "config_params/params.json"

selector = mode_switch.selector
device_parameters = (
    config.DeviceParameters(selector.device_id)
)

robot = CMBenchTop(_PARAM_FILE, "CubeMars_BenchTop",
                   device_parameters)
# robot = FoxKnee(_PARAM_FILE, "foxKnee_KneeExo",
#                    device_parameters)

ctrlFact = CtrlFactory(robot, _PARAM_FILE)

netManager = NetworkManager()
IP = netManager.connect_WLAN()
print("Personal IP:"+ IP)

# this one is the IP setted in the PC (I used two different one for the 2 pc tested)
reporter_ip = "192.168.0.252"
# reporter_ip = "192.168.0.251"
# # input manager for receiving control input:
inputMan = IMTCP(ctrlFact=ctrlFact, receiver_ip=IP)
# reporter for streaming outside relevant topics
reporter = ReporterWifiUDP(ctrlFact, _PARAM_FILE, reporter_ip)
reporter.enable_reporter()

robot.enable(True)
robot.set_backemf_feedforward(True)   # remove this line to test WITHOUT feedforward









