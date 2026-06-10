#!/usr/bin/env python3

import socket
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# TODO: do the board the server and this one client?

class tcp2ROS(Node):
    def __init__(self):
        # define ros interfaces
        super().__init__('tcp2ROS')
        self.declare_parameter('robot', 'CubeMars')
        self.declare_parameter('host_address', '192.168.0.251')
        self.declare_parameter('port', 12345)

        robot = self.get_parameter('robot').get_parameter_value().string_value
        host_address = self.get_parameter('host_address').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        #from my_multiarray_topic to robot and control state topics
        self.pub_robot = self.create_publisher(Float64MultiArray, f'robot_state_{robot}', 10)   
        self.pub_ctrl = self.create_publisher(Float64MultiArray, f'ctrl_state_{robot}', 10)
    
        # define socket interfaces
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind((host_address, port))
        self.s.listen(1) # TODO: check these
        self.s.settimeout(0.01)
        print(f"Listening on {host_address}:{port}")

        # quantify stats
        self.last_time = None
        self.packet_count = 0
        self.total_time = 0.0
        self.last_index = None
        self.lost_packets = 0
        self.last_pub_time = time.time()
        self.pub_count = 0

        # Dimension mapping
        self.dim_reporter_msg = 1
        self.dim_ctrl_fact_msg = 1
        self.dim_ctrl_msg = 10
        self.dim_dec_msg = 5
        self.dim_robot_msg = 11

    def _log_stats(self):
        if self.packet_count > 0:
            avg_freq = self.packet_count / self.total_time if self.total_time > 0 else 0
            avg_pub_freq = self.pub_count / (time.time() - self.last_pub_time)
            self.get_logger().info(f"[Stats] Rx freq: {avg_freq:.1f} Hz | Lost: {self.lost_packets} | ROS pub freq: {avg_pub_freq:.1f} Hz")
            # reset
            self.packet_count = 0
            self.total_time = 0
            self.pub_count = 0
            self.last_pub_time = time.time()

    def main_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            try:
                # print("waiting connection!")
                self.conn, addr = self.s.accept() # new socket is conn
                self.conn.settimeout(0.01)
                self.get_logger().info(f"Connected by {addr}")
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break

            buffer = ""
            try:
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0)
                    try:
                        data = self.conn.recv(1024) # check this
                        if not data:
                            self.get_logger().info("Connection closed by client --> waiting new client")
                            break
                        else: 
                            buffer += data.decode()
                            now = time.time()
                            if self.last_time:
                                self.total_time += (now - self.last_time)
                            self.last_time = now
                            self.packet_count += 1
                            # print(buffer)
                    except socket.timeout:
                        # no client connected within timeout — loop again
                        pass
                    except KeyboardInterrupt:
                        break
                    
                    # process buffer if any
                    while "\n" in buffer:
                        msg, buffer = buffer.split("\n", 1)
                        # print(buffer)

                        if not msg.strip():
                            continue
                        try:
                            msg_float = [float(v) for v in msg.split(",")]
                            msg_ros = Float64MultiArray(data=msg_float)
                            
                            # Slicer 
                            start_idx = 0
                            msg_header = msg_float[start_idx : start_idx + self.dim_reporter_msg]
                            start_idx += self.dim_reporter_msg

                            msg_ctrl_fact = msg_float[start_idx : start_idx + self.dim_ctrl_fact_msg]
                            start_idx += self.dim_ctrl_fact_msg

                            msg_ctrl = msg_float[start_idx : start_idx + self.dim_ctrl_msg]
                            start_idx += self.dim_ctrl_msg

                            msg_dec = msg_float[start_idx : start_idx + self.dim_dec_msg]
                            start_idx += self.dim_dec_msg

                            msg_robot = msg_float[start_idx : start_idx + self.dim_robot_msg]
                            
                            final_robot = msg_header + msg_robot
                            final_ctrl = msg_header + msg_ctrl

                            msg_ros_robot = Float64MultiArray()
                            msg_ros_robot.data = final_robot
                            self.pub_robot.publish(msg_ros_robot)

                            msg_ros_ctrl = Float64MultiArray()
                            msg_ros_ctrl.data = final_ctrl
                            self.pub_ctrl.publish(msg_ros_ctrl)

                            # --- Metrics ---
                            self.pub_count += 1

                            # Check for packet loss (assuming first value is index)
                            if self.last_index is not None and len(msg_float) > 0:
                                idx = int(msg_float[0])
                                if idx != self.last_index + 1:
                                    self.lost_packets += (idx - self.last_index - 1)
                                self.last_index = idx
                            else:
                                if len(msg_float) > 0:
                                    self.last_index = int(msg_float[0])

                            # print stats every 5s
                            if time.time() - self.last_pub_time > 5:
                                self._log_stats()
                        
                        except KeyboardInterrupt:
                            break
                        except ValueError:
                            self.get_logger().warn(f"Invalid data: {msg}")
                            # skip, don't close the socket
                            continue
                        
            except OSError as e:
                self.get_logger().warn(f"Socket error: {e}")
            finally:
                self.conn.close()
                print("Connection ended, waiting for new client...")


  
def main(args=None):
    rclpy.init(args=args)
    t2R = tcp2ROS()
    t2R.main_loop()
    t2R.s.close()
    t2R.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()