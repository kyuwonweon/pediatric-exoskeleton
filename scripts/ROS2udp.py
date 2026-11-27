#!/usr/bin/env python3

import socket
import json
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from std_srvs.srv import Trigger

from micropython_logger.ctrl_configs import (
    GuiOscillatorPos,
    GuiOscillatorForce,
    GuiFFCal,
    GuiFBTransp,
    GuiTeleop,
    GuiSMWalking,
    GuiSMWalking3States,
)
from micropython_logger.reconfigure import ReconfigureServer

"""
This class is design for fast comunication
in particular this class manages comunication of the kind
rostopic --> udp
the only exeption (at the moment) is to add the ros service for enabling and change K and B for the teloep

Includes the devel/CMbenchTop (ef24346) changes: SMWalking3States / SMWalking4States,
a command length that follows the controller (20 or 23 fields), and bounded packing.
"""


class ROS2udp(Node):
    def __init__(self):
        super().__init__('ros2_udp')
        self.declare_parameter('robot', 'Tails')
        self.declare_parameter('ctrl', 'Oscillator')

        robot = self.get_parameter('robot').get_parameter_value().string_value
        self.ctrl = self.get_parameter('ctrl').get_parameter_value().string_value

        self.get_logger().info(f"Robot: {robot}")
        self.get_logger().info(f"Ctrl: {self.ctrl}")

        self.port = 12345

        if robot == "Tails":
            # self.receiver_ip= '192.168.0.68'
            # self.receiver_ip= '192.168.0.172'  # Lorenzo's network (devel/CMbenchTop)
            self.receiver_ip = '192.168.0.202' # in the case in which tails is the access point
        elif robot == "Shadow":
            self.receiver_ip = '192.168.0.254'
        else:
            raise Exception("Wrong Robot assignement")
        # Optional override, e.g. a bench board on another address or a local test server.
        self.receiver_ip = self.declare_parameter('receiver_ip', self.receiver_ip).value
        self.get_logger().info(f"Connecting to IP: {self.receiver_ip}, Port: {self.port}")

        ctrl_msg_dim = 20
        header = 0
        config_params = []
        if self.ctrl == "Teleop":
            header = 3
            config_params = GuiTeleop
        elif self.ctrl == "SMWalking":
            header = 4
            if robot == "CubeMars":
                config_params = GuiSMWalking3States
                ctrl_msg_dim = 20
            else:
                config_params = GuiSMWalking
                ctrl_msg_dim = 23
        elif self.ctrl == "SMWalking3States":
            header = 4
            config_params = GuiSMWalking3States
            ctrl_msg_dim = 20
        elif self.ctrl == "SMWalking4States":
            header = 4
            config_params = GuiSMWalking
            ctrl_msg_dim = 23
        elif self.ctrl == "OscillatorPos":
            header = 6
            config_params = GuiOscillatorPos
        elif self.ctrl == "OscillatorForce":
            header = 5
            config_params = GuiOscillatorForce
        elif self.ctrl == "FF":
            header = 1
            config_params = GuiFFCal
        elif self.ctrl == "FB":
            header = 2
            config_params = GuiFBTransp
        else:
            self.get_logger().error(f"Unknown controller mode: {self.ctrl}")

        # Field position in the command vector, in .cfg order.
        self.param_idx_map = {spec.name: index for index, spec in enumerate(config_params)}

        # define socket interfaces
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # wait a second the socket to start
        time.sleep(1)

        self.header_msg = np.array([header])
        self.ctrl_msg = np.zeros(ctrl_msg_dim)
        # robot msg:
        # list[int], by default all zero
        # if index 0 ==1 --> imu calibration
        # elif index 2 ==1 --> load-cell calibration[int]
        self.robot_msg = np.zeros(4)

        if self.ctrl == "Teleop":
            # subscriber to the ros node containing the message to convert to udp
            self.sub_teleop = self.create_subscription(Float64MultiArray,
                                                       f'ctrl_command_{robot}',
                                                       self.callback_telop_subs, 10)
            self.get_logger().info("Teleop mode active.")

        self.server = ReconfigureServer(self, config_params, self.param_update_callback)
        self.service_IMU_Cal = self.create_service(Trigger, f'{robot}/IMU_calibration', self.imu_cal_callback_)
        self.service_Load_Cel_cal = self.create_service(Trigger, f'{robot}/LoadCell_calibration', self.LoadCell_cal_callback_)

    def callback_telop_subs(self, msg):
        """Update with teleop changes"""
        try:
            rp = list(msg.data)

            enable_idx = self.param_idx_map.get("enable_controller", 0)
            k_idx = self.param_idx_map.get("K_stiffness", 3)
            b_idx = self.param_idx_map.get("B_damping", 4)

            rp[0] = self.ctrl_msg[enable_idx]
            rp[3] = self.ctrl_msg[k_idx]
            rp[4] = self.ctrl_msg[b_idx]

            out_str = ",".join(str(x) for x in rp) + "\n"
            # print(msg)
            self.s.sendto(out_str.encode(), (self.receiver_ip, self.port))

        except AttributeError as e:
            print(e)
            print("Client not yet connected")
        except socket.timeout as e:
            print(e)
            print("Client not yet connected")
        except BrokenPipeError as e:
            print(e)
            print("Client not yet connected")

    def imu_cal_callback_(self, req, res):
        self.get_logger().info("IMU Cal called!")
        self.robot_msg[0] = 1.0
        success = self.send_and_Ack_msg()
        res.success = success
        if success:
            res.message = "IMU cal completed successfully."
        else:
            res.message = "IMU cal Not completed."
        # restore default value
        self.robot_msg[0] = 0.0
        return res

    def LoadCell_cal_callback_(self, req, res):
        self.get_logger().info("LOAD cell cal called!")
        self.robot_msg[1] = 1.0
        success = self.send_and_Ack_msg()
        res.success = success
        if success:
            res.message = "Load cell cal completed successfully."
        else:
            res.message = "Load cell cal Not completed."
        self.robot_msg[1] = 0.0
        return res

    def send_and_Ack_msg(self):
        success = False

        try:
            # build payload
            rp = np.zeros(1 + len(self.ctrl_msg) + len(self.robot_msg), dtype=np.float32)
            rp[0] = self.header_msg[0]
            rp[1:1 + len(self.ctrl_msg)] = self.ctrl_msg
            rp[1 + len(self.ctrl_msg):] = self.robot_msg

            msg = rp.tobytes() + b'\n' # no newline needed for UDP

            # send
            start = time.perf_counter()
            self.s.sendto(msg, (self.receiver_ip, self.port))

            # wait for ACK
            self.s.settimeout(0.1)  # 100 ms
            data, addr = self.s.recvfrom(128)

            if data == msg:
                elapsed_ms = (time.perf_counter() - start) * 1000
                print(f"UDP RTT: {elapsed_ms:.2f} ms")
                success = True

        except socket.timeout:
            self.get_logger().warning("UDP ACK timeout")
        except Exception as e:
            self.get_logger().error(f"UDP error:{e}")

        return success

    def param_update_callback(self, config, level):
        # Same packing as the ROS1 dynamic_reconfigure callback: fields in .cfg order.
        for name, idx in self.param_idx_map.items():
            if idx < len(self.ctrl_msg):
                self.ctrl_msg[idx] = float(config[name])
        self.get_logger().info(f"Updated {self.ctrl} parameters: {self.ctrl_msg.tolist()}")
        if self.ctrl != "Teleop":
            self.send_and_Ack_msg()
        return config


def main(args=None):
    rclpy.init(args=args)
    r2u = ROS2udp()
    try:
        rclpy.spin(r2u)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        r2u.s.close()
        r2u.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
