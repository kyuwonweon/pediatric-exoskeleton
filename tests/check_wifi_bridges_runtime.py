"""End-to-end check of the Wi-Fi bridges (ROS2tcp, ROS2udp, udp2ROS) with fake boards.

Covers the devel/CMbenchTop (ef24346) changes ported to ROS2: the SMMultipleAct
controller over TCP (bulk push, FromTopic activity, save/load), the 4-state
SMWalking packet length over UDP, the Teleop header, and the udp2ROS rewrite.
Runs in an isolated ROS domain on localhost. No boards or motors are used.

    source install/setup.bash
    python3 tests/check_wifi_bridges_runtime.py
"""

import os
import random
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path

ROOT = Path(tempfile.mkdtemp(prefix='wifi_bridges_runtime_'))
print('Isolated ROS test logs:', ROOT, flush=True)
os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_HOME'] = str(ROOT / 'home')
os.environ['ROS_LOG_DIR'] = str(ROOT / 'logs')
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

BOARD_PORT = 12345  # fixed in ROS2tcp / ROS2udp


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_until(predicate, timeout=15.0, what='condition'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('Timed out waiting for ' + what)


class FakeTcpBoard:
    """Echo every float32 command back, as the board firmware does for its ACK."""

    def __init__(self, floats_per_msg):
        self.size = 4 * floats_per_msg
        self.messages = []
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', BOARD_PORT))
        self.server.listen(1)
        self.server.settimeout(0.2)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        conn = None
        while not self.stop.is_set() and conn is None:
            try:
                conn, _ = self.server.accept()
            except socket.timeout:
                continue
        conn.settimeout(0.2)
        buffer = b''
        while not self.stop.is_set():
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            buffer += chunk
            while len(buffer) >= self.size:
                msg, buffer = buffer[:self.size], buffer[self.size:]
                self.messages.append(struct.unpack('<{}f'.format(self.size // 4), msg))
                conn.sendall(msg)
        conn.close()

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.server.close()


class FakeUdpBoard:
    def __init__(self):
        self.messages = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', BOARD_PORT))
        self.sock.settimeout(0.2)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                data, sender = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            payload = data[:-1] if data.endswith(b'\n') and len(data) % 4 == 1 else data
            self.messages.append(struct.unpack('<{}f'.format(len(payload) // 4), payload))
            self.sock.sendto(data, sender)

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.sock.close()


processes = []
logs = []


def start_node(name, executable, params):
    output = (ROOT / (name + '.log')).open('w')
    logs.append(output)
    args = ['ros2', 'run', 'micropython_logger', executable, '--ros-args']
    for key, value in params.items():
        args += ['-p', '{}:={}'.format(key, value)]
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(process)
    return process


def stop_process(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

rclpy.init()
node = rclpy.create_node('wifi_bridges_runtime_check')
executor = MultiThreadedExecutor()
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()


def set_params(target, **values):
    client = node.create_client(SetParametersAtomically, target + '/set_parameters_atomically')
    try:
        assert client.wait_for_service(timeout_sec=15), target
        request = SetParametersAtomically.Request(
            parameters=[Parameter(k, value=v).to_parameter_msg() for k, v in values.items()])
        result = client.call(request, timeout_sec=30).result
        assert result.successful, result.reason
    finally:
        node.destroy_client(client)


def call(service):
    client = node.create_client(Trigger, service)
    try:
        assert client.wait_for_service(timeout_sec=15), service
        return client.call(Trigger.Request(), timeout_sec=30)
    finally:
        node.destroy_client(client)


boards = []
try:
    # ---- ROS2tcp, SMMultipleAct (header 7, 1 + 24 + 4 floats) ----
    board = FakeTcpBoard(29)
    boards.append(board)
    tcp = start_node('ros2tcp_multiact', 'ROS2tcp.py', {
        'robot': 'CubeMars', 'ctrl': 'SMMultipleAct', 'receiver_ip': '127.0.0.1',
        'locomotion_topic': '/test_committed_mode',
    })
    call('/IMU_calibration')  # services exist once the startup push has finished
    assert all(msg[0] == 7 for msg in board.messages)
    pairs = {(int(msg[2]), int(msg[3])) for msg in board.messages}
    assert pairs >= {(a, p) for a in range(5) for p in range(4)}, pairs
    assert all(msg[1] == 0 for msg in board.messages), 'startup must not enable the controller'
    print('PASS: ROS2tcp SMMultipleAct pushes all 20 (activity, phase) profiles disabled at startup', flush=True)

    imu = [msg for msg in board.messages if msg[25] == 1]
    assert len(imu) == 1 and board.messages[-1][25] == 1
    print('PASS: ROS2tcp IMU calibration sends robot_msg[0]=1 and is ACKed', flush=True)

    before = len(board.messages)
    set_params('/ros2_tcp', activity=2, phase=1)
    set_params('/ros2_tcp', K=0.7)
    wait_until(lambda: any(abs(m[7] - 0.7) < 1e-6 for m in board.messages[before:]), what='K edit')
    sent = board.messages[-1]
    assert (int(sent[2]), int(sent[3]), round(sent[7], 3)) == (2, 1, 0.7), sent
    print('PASS: ROS2tcp profile edit for StairsDesc/LateStance reaches the board', flush=True)

    publisher = node.create_publisher(
        String, '/test_committed_mode', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    set_params('/ros2_tcp', activity_source=1)
    before = len(board.messages)
    publisher.publish(String(data='ramp_up'))
    wait_until(lambda: any(int(m[2]) == 3 for m in board.messages[before:]), what='FromTopic ramp_up')
    print('PASS: FromTopic_ComputerVision maps ramp_up to RampAsc and sends it', flush=True)

    set_params('/ros2_tcp', save_name='wifi_test')
    assert call('/SaveSMParams').success
    saved = ROOT / 'home/sm_multi_act_params/saved/wifi_test.yaml'
    assert saved.is_file(), saved
    set_params('/ros2_tcp', activity_source=0, load_name='SM_AllOW')
    response = call('/LoadSMParams')
    assert response.success and 'share' in response.message, response
    print('PASS: SaveSMParams writes under ROS_HOME; LoadSMParams finds the shipped SM_AllOW snapshot', flush=True)
    stop_process(tcp)
    board.close()

    # ---- ROS2udp, 4-state SMWalking (header 4, 1 + 23 + 4 floats) ----
    board = FakeUdpBoard()
    boards.append(board)
    udp = start_node('ros2udp_sm4', 'ROS2udp.py', {
        'robot': 'Tails', 'ctrl': 'SMWalking', 'receiver_ip': '127.0.0.1',
    })
    wait_until(lambda: board.messages, what='ROS2udp startup message')
    assert len(board.messages[0]) == 28 and board.messages[0][0] == 4, board.messages[0]
    set_params('/ros2_udp', lateSwing_B=0.25)
    # Packet = header + 23 fields in .cfg order; lateSwing_B is field 20 -> packet index 21.
    wait_until(lambda: abs(board.messages[-1][21] - 0.25) < 1e-6, what='lateSwing_B edit')
    print('PASS: ROS2udp SMWalking for Tails uses the 4-state, 28-float packet', flush=True)
    stop_process(udp)

    udp = start_node('ros2udp_teleop', 'ROS2udp.py', {
        'robot': 'Tails', 'ctrl': 'Teleop', 'receiver_ip': '127.0.0.1',
    })
    before = len(board.messages)
    assert call('/Tails/IMU_calibration').success
    teleop = board.messages[before:]
    assert teleop and teleop[-1][0] == 3, teleop
    print('PASS: ROS2udp Teleop packets carry header 3 (was overwritten to 0)', flush=True)
    stop_process(udp)
    board.close()

    # ---- udp2ROS ----
    listen_port = free_port()
    u2r = start_node('udp2ros', 'udp2ROS.py', {
        'robot': 'CubeMars', 'listen_host': '127.0.0.1', 'listen_port': listen_port,
    })
    received = []
    node.create_subscription(Float64MultiArray, '/robot_state_CubeMars', lambda m: received.append(list(m.data)), 10)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = struct.pack('<27f', *range(27))
    wait_until(lambda: (sender.sendto(packet, ('127.0.0.1', listen_port)), received)[1], what='udp2ROS publish')
    assert received[0] == [0.0] + [float(v) for v in range(17, 27)], received[0]
    print('PASS: udp2ROS unpacks a 27-float Wi-Fi report into robot_state_CubeMars', flush=True)
    stop_process(u2r)
finally:
    for process in processes:
        stop_process(process)
    for board in boards:
        try:
            board.close()
        except Exception:
            pass
    for output in logs:
        output.close()
    rclpy.try_shutdown()

print('All Wi-Fi bridge checks passed. No boards or motors were used.', flush=True)
