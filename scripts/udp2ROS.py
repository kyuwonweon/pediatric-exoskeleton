#!/usr/bin/env python3
"""Receive legacy MicroPython Wi-Fi float arrays and publish ROS topics.

Includes the devel/CMbenchTop (ef24346) rewrite: listen_host / listen_port
parameters, stats even while idle, and an empty datagram no longer ends the loop.
"""

import socket
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class udp2ROS(Node):
    """Receive legacy MicroPython Wi-Fi float arrays and publish ROS topics."""

    PORT_BY_ROBOT = {
        "Tails": 12345,
        "Shadow": 12344,
        "CubeMars": 12346,
    }

    def __init__(self):
        super().__init__("udp2ROS")
        self.get_logger().info("ROS Init success")

        self.robot = self.declare_parameter("robot", "CubeMars").value
        # host_address is the computer address; listen_host overrides it (devel/CMbenchTop name).
        host_address = self.declare_parameter("host_address", "192.168.0.252").value
        self.pub_robot = self.create_publisher(Float64MultiArray, "robot_state_" + self.robot, 10)
        self.pub_ctrl = self.create_publisher(Float64MultiArray, "ctrl_state_" + self.robot, 10)

        # dimensions of the messages (these are defined in the json inside the board)
        self.dim_robot_msg = 10
        self.dim_ctrl_msg = 10
        self.dim_dec_msg = 5
        self.dim_ctrl_fact_msg = 1
        self.dim_reporter_msg = 1
        self.len_msg = (
            self.dim_robot_msg
            + self.dim_ctrl_msg
            + self.dim_dec_msg
            + self.dim_ctrl_fact_msg
            + self.dim_reporter_msg
        )

        listen_host = self.declare_parameter("listen_host", host_address).value
        default_port = self.PORT_BY_ROBOT.get(self.robot, 12346)
        listen_port = int(self.declare_parameter("listen_port", default_port).value)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.settimeout(0.02)
        self.s.bind((listen_host, listen_port))
        self.get_logger().info("Listening (wifi_udp) on {}:{}".format(listen_host, listen_port))

        # quantify stats
        self.last_time = None
        self.packet_count = 0
        self.total_time = 0.0
        self.last_index = None
        self.lost_packets = 0
        self.last_pub_time = time.time()
        self.pub_count = 0
        self.receiving_addr = ""

    def _publish_vector(self, values):
        start_index = 0
        msg_header = [
            float(v)
            for v in values[
                start_index:start_index + self.dim_reporter_msg
            ]
        ]
        start_index += self.dim_reporter_msg

        start_index += self.dim_ctrl_fact_msg
        msg_ctrl = [
            float(v)
            for v in values[start_index:start_index + self.dim_ctrl_msg]
        ]
        start_index += self.dim_ctrl_msg

        start_index += self.dim_dec_msg
        msg_robot = [
            float(v)
            for v in values[start_index:start_index + self.dim_robot_msg]
        ]

        self.pub_robot.publish(Float64MultiArray(data=msg_header + msg_robot))
        self.pub_ctrl.publish(Float64MultiArray(data=msg_header + msg_ctrl))
        self.pub_count += 1

        if len(values) > 0:
            self._update_loss_stats(int(values[0]))

    def _update_loss_stats(self, current_index):
        if self.last_index is not None and current_index > self.last_index + 1:
            self.lost_packets += current_index - self.last_index - 1
        self.last_index = current_index

    def _process_packet(self, data):
        expected_len = 4 * self.len_msg
        if len(data) != expected_len:
            self.get_logger().warning(
                "Invalid Wi-Fi packet size: {} bytes (expected {})".format(
                    len(data), expected_len
                )
            )
            return

        values = np.frombuffer(data, dtype=np.float32)
        self._publish_vector(values)

    def _log_stats(self):
        if self.packet_count > 0:
            rx_frequency = (
                self.packet_count / self.total_time
                if self.total_time > 0
                else 0.0
            )
            elapsed = max(time.time() - self.last_pub_time, 1e-6)
            pub_frequency = self.pub_count / elapsed
            self.get_logger().info(
                "[Stats] Rec from addr: {} | Rx freq: {:.1f} Hz | "
                "Lost: {} | ROS pub freq: {:.1f} Hz".format(
                    self.receiving_addr,
                    rx_frequency,
                    self.lost_packets,
                    pub_frequency,
                )
            )

        self.packet_count = 0
        self.total_time = 0.0
        self.pub_count = 0
        self.last_pub_time = time.time()

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            try:
                data, self.receiving_addr = self.s.recvfrom(4096)
                if not data:
                    continue

                now = time.time()
                if self.last_time is not None:
                    self.total_time += now - self.last_time
                self.last_time = now
                self.packet_count += 1

                self._process_packet(data)
                if time.time() - self.last_pub_time > 5.0:
                    self._log_stats()

            except socket.timeout:
                if time.time() - self.last_pub_time > 5.0:
                    self._log_stats()
            except KeyboardInterrupt:
                break
            except OSError as error:
                self.get_logger().warning("Socket error: {}".format(error))
                time.sleep(0.2)
            except Exception as error:
                self.get_logger().warning("Runtime error: {}".format(error))
                time.sleep(0.05)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    bridge = udp2ROS()
    try:
        bridge.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()
        bridge.destroy_node()
        rclpy.try_shutdown()
        print("Connection ended, waiting for new client...")


if __name__ == "__main__":
    main()
