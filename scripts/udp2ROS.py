#!/usr/bin/env python3

import socket
import json
import time
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64

# host address is the computer address
# port is dependent by which robot is transmitting
class udp2ROS(Node):
    def __init__(self):
        super().__init__('udp2ROS')

        self.declare_parameter('robot', 'CubeMars')
        self.declare_parameter('host_address', '192.168.0.252')

        robot = self.get_parameter('robot').get_parameter_value().string_value
        host_address = self.get_parameter('host_address').get_parameter_value().string_value

        self.get_logger().info("ROS Init success")

        if robot =="Tails": port= 12345
        elif robot == "Shadow": port = 12344
        elif robot == "CubeMars": port = 12346

        self.pub_robot = self.create_publisher(Float64MultiArray, f'robot_state_{robot}', 10)
        self.pub_ctrl = self.create_publisher(Float64MultiArray, f'ctrl_state_{robot}', 10)
        self.pub_backemf = self.create_publisher(Float64, f'backemf_{robot}', 10)

        # define socket interfaces
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host_address, port))
        self.s.settimeout(0.001)
        self.get_logger().info(f"Listening on {host_address} from port {port}")

        # quantify stats
        self.last_time = None
        self.packet_count = 0
        self.total_time = 0.0
        self.last_index = None
        self.lost_packets = 0
        self.last_pub_time = time.time()
        self.pub_count = 0
        self.receiving_addr = ""
        self.new_msg_flag = False

        # dimensions of the messages (these are defined in the json inside the board) 
        # TODO: potentially at one point copy json
        self.dim_robot_msg = 11
        self.dim_ctrl_msg = 10
        self.dim_dec_msg = 5
        self.dim_ctrl_fact_msg =1
        self.dim_reporter_msg = 1
        self.len_msg = (self.dim_robot_msg + self.dim_ctrl_msg + 
                        self.dim_dec_msg + self.dim_ctrl_fact_msg + self.dim_reporter_msg)

    def _log_stats(self):
        if self.packet_count > 0:
            avg_freq = self.packet_count / self.total_time if self.total_time > 0 else 0
            avg_pub_freq = self.pub_count / (time.time() - self.last_pub_time)
            self.get_logger().info(f"[Stats] Rec from addr: {self.receiving_addr} | Rx freq: {avg_freq:.1f} Hz | Lost: {self.lost_packets} | ROS pub freq: {avg_pub_freq:.1f} Hz")
            # reset
            self.packet_count = 0
            self.total_time = 0
            self.pub_count = 0
            self.last_pub_time = time.time()

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            try:
                data, self.receiving_addr = self.s.recvfrom(1024) # check this
                # self.s.sendto(data, self.receiving_addr)
                if not data:
                    print("Connection closed by client --> waiting new client")
                    break
                else: 
                    now = time.time()
                    if self.last_time:
                        self.total_time += (now - self.last_time)
                    self.last_time = now
                    self.packet_count += 1

                    expected_len = 4 * self.len_msg
                    if len(data) != expected_len:
                        self.get_logger().warn(f"Invalid data size: {len(data)} bytes")
                        continue
                    self.new_msg_flag = True

                    # TODO: maybe add a quality check indicator (for instance one of the values should be always constant)
                    msg_splitted = np.frombuffer(data, dtype=np.float32)
                    start_index = 0

                    msg_header = [float(v) for v in msg_splitted[start_index: start_index + self.dim_reporter_msg]] # reporter header
                    start_index += self.dim_reporter_msg
                    # ctrl Fact
                    msg_ctrl_fact = [float(v) for v in msg_splitted[start_index: start_index + self.dim_ctrl_fact_msg]]
                    start_index += self.dim_ctrl_fact_msg
                    # CTRL  
                    msg_ctrl = [float(v) for v in msg_splitted[start_index: start_index + self.dim_ctrl_msg]]
                    start_index += self.dim_ctrl_msg
                    # decorator
                    msg_dec = [float(v) for v in msg_splitted[start_index: start_index + self.dim_dec_msg]]
                    start_index += self.dim_dec_msg

                    msg_robot =  [float(v) for v in msg_splitted[start_index: (start_index+ self.dim_robot_msg)]]
                    
                    # add header to both messages
                    msg_robot = msg_header + msg_robot
                    msg_ctrl = msg_header + msg_ctrl

                    msg_ros_robot = Float64MultiArray()
                    msg_ros_robot.data = msg_robot
                    msg_ros_ctrl = Float64MultiArray()
                    msg_ros_ctrl.data = msg_ctrl
                    bemf = Float64()
                    bemf.data = float(msg_robot[self.dim_reporter_msg + 9]) # robot slot 9 = back-EMF (Kt*omega)
                    self.pub_robot.publish(msg_ros_robot)
                    self.pub_ctrl.publish(msg_ros_ctrl)
                    self.pub_backemf.publish(bemf)
                    self.new_msg_flag =  False

                    # --- Metrics ---
                    self.pub_count += 1

                    # Check for packet loss (assuming first value is index)
                    if self.last_index is not None and len(msg_ctrl) > 0:
                        idx = int(msg_header[0])
                        if idx != self.last_index + 1:
                            self.lost_packets += (idx - self.last_index - 1)
                        self.last_index = idx
                    else:
                        if len(msg_ctrl) > 0:
                            self.last_index = int(msg_header[0])

                    # print stats every 5s
                    if time.time() - self.last_pub_time > 5:
                        self._log_stats()

            except socket.timeout:
                # no client connected within timeout — loop again
                pass
            except Exception as e:
                self.get_logger().error(f"Error parsing data: {e}")


def main(args=None):
    rclpy.init(args=args)
    u2R = udp2ROS()
    try:
        u2R.main_loop()
    except KeyboardInterrupt:
        pass
    u2R.s.close()
    u2R.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
  
if __name__ == "__main__":
    main()
