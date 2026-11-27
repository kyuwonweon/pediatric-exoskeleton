"""End-to-end check of the CubeMars CAN launch with simulated boards.

ROS2 port of tests/check_can_multi_activity_runtime.py (devel/CMbenchTop ef24346).
Runs the real launch file, ROS2canIP, CANMotorGroup and canIP2ROS nodes against
a fake canIP/relay on localhost, in an isolated ROS domain. No CAN or motors.

    source install/setup.bash
    python3 tests/check_can_multi_activity_runtime.py
"""

import os
import random
import signal
import socket
import struct
import subprocess
import threading
import tempfile
import time
from pathlib import Path


ROOT = Path(tempfile.mkdtemp(prefix='cubemars_can_runtime_'))
print('Isolated ROS test logs:', ROOT, flush=True)


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def wait_for_configuration(client, expected):
    # Kept from the ROS1 check; ROS2 parameter reads are direct, so this returns at once.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        config = client.get_configuration(timeout=0.2)
        if config is not None and all(config[key] == value for key, value in expected.items()):
            return
        time.sleep(0.01)
    raise AssertionError('Configuration update not observed: {}'.format(expected))


class FakeRelay:
    def __init__(self, relay_mode=True):
        self.relay_mode = relay_mode
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', 0))
        self.socket.settimeout(0.1)
        self.port = self.socket.getsockname()[1]
        self.stop = threading.Event()
        self.samples = []
        self.pending = {}
        self.reject_initial_node2 = True
        self.drop_next_node2_start = False
        self.drop_next_node2_activity = False
        self.require_both_start = True
        self.held_start_acks = []
        self.saw_concurrent_start = False
        self.error = None
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set():
                try:
                    packet, sender = self.socket.recvfrom(4096)
                except socket.timeout:
                    continue
                if packet == b'hello':
                    if self.relay_mode:
                        self.socket.sendto(b'CANIP_RELAY_OK', sender)
                    continue
                assert len(packet) == 12 and packet[0] == 0x65
                assert packet[-1] == sum(packet[1:-1]) & 255
                sid = ((packet[1] & 7) << 8) | packet[2]
                node, index = divmod(sid - 0x300, 32)
                payload = packet[3:11]
                seq, payload_index, count, _ = payload[:4]
                assert payload_index == index and count == 25
                state = self.pending.setdefault((node, seq), {})
                state[index] = payload[4:8]
                if self.relay_mode:
                    self.socket.sendto(b'CANIP_RELAY_OK', sender)
                if len(state) != count:
                    continue
                raw = b''.join(state[i] for i in range(count))
                values = struct.unpack('<25f', raw)
                enabled = values[1] != 0
                self.samples.append((node, seq, enabled, values, time.monotonic()))
                self.pending.pop((node, seq))
                checksum = 2166136261
                for byte in raw:
                    checksum = ((checksum ^ byte) * 16777619) & 0xFFFFFFFF
                status = 1
                if node == 2 and not enabled and self.reject_initial_node2:
                    self.reject_initial_node2 = False
                    status = 2
                response = ('CANIP_RELAY_CAN_ACK node={} can_id={} seq={} status={} count=25 ctrl={} checksum={} expected_checksum={} checksum_match=1'.format(
                    node, 0x300 + node * 32 + 31, seq, status, int(values[0]), checksum, checksum
                )).encode('ascii')
                if not self.relay_mode:
                    ack_id = 0x300 + node * 32 + 31
                    ack_data = struct.pack('<BBBBI', seq, status, 25, int(values[0]), checksum)
                    hdr1 = (8 << 3) | (ack_id >> 8)
                    hdr2 = ack_id & 255
                    response = bytes([0x65, hdr1, hdr2]) + ack_data + bytes([(hdr1 + hdr2 + sum(ack_data)) & 255])
                if node == 2 and enabled and self.drop_next_node2_start:
                    self.drop_next_node2_start = False
                    continue
                if node == 2 and self.drop_next_node2_activity:
                    self.drop_next_node2_activity = False
                    continue
                if enabled and self.require_both_start:
                    self.held_start_acks.append((response, sender))
                    if len(self.held_start_acks) == 2:
                        self.saw_concurrent_start = True
                        self.require_both_start = False
                        for ack, target in self.held_start_acks:
                            self.socket.sendto(ack, target)
                        self.held_start_acks = []
                else:
                    self.socket.sendto(response, sender)
        except Exception as error:
            self.error = error

    def close(self):
        self.stop.set()
        self.thread.join(2)
        self.socket.close()
        assert self.error is None, self.error


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


# A private DDS domain on localhost replaces the ROS1 test's private roscore.
os.environ['ROS_DOMAIN_ID'] = str(random.randint(100, 200))
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['ROS_HOME'] = str(ROOT / 'home')
os.environ['ROS_LOG_DIR'] = str(ROOT / 'logs')
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'

launch = None
relay = None
canip = None
logs = []
rclpy = None
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rcl_interfaces.srv import GetParameters, ListParameters, SetParametersAtomically
    from std_srvs.srv import Trigger

    rclpy.init()
    test_node = rclpy.create_node('cubemars_launch_runtime_check')
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    class Client:
        """The parts of dynamic_reconfigure.client.Client the ROS1 check used."""

        def __init__(self, node_name, timeout):
            self.node_name = node_name
            self._set = test_node.create_client(SetParametersAtomically, node_name + '/set_parameters_atomically')
            self._get = test_node.create_client(GetParameters, node_name + '/get_parameters')
            self._list = test_node.create_client(ListParameters, node_name + '/list_parameters')
            for client in (self._set, self._get, self._list):
                assert client.wait_for_service(timeout_sec=timeout), node_name
            names = self._list.call(ListParameters.Request(), timeout_sec=timeout).result.names
            self._names = [name for name in names if name != 'use_sim_time']

        def get_configuration(self, timeout=5.0):
            response = self._get.call(GetParameters.Request(names=self._names), timeout_sec=timeout)
            return {name: parameter_value_to_python(value) for name, value in zip(self._names, response.values)}

        def update_configuration(self, changes):
            if changes:
                request = SetParametersAtomically.Request(
                    parameters=[Parameter(name, value=value).to_parameter_msg() for name, value in changes.items()]
                )
                result = self._set.call(request, timeout_sec=30.0).result
                assert result.successful, (self.node_name, changes, result.reason)
            return self.get_configuration()

        def close(self):
            for client in (self._set, self._get, self._list):
                test_node.destroy_client(client)

    relay = FakeRelay()

    def start_launch(name, extra):
        output = (ROOT / (name + '.log')).open('w')
        logs.append(output)
        return subprocess.Popen([
            'ros2', 'launch', 'micropython_logger', 'cubemars_can.launch.py',
            'start_receiver:=false', 'relay_port:={}'.format(relay.port),
            'command_ack_timeout_s:=0.5', 'service_wait_timeout_s:=2.0',
        ] + extra, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)

    def call(service):
        client = test_node.create_client(Trigger, service)
        try:
            assert client.wait_for_service(timeout_sec=15), service
            return client.call(Trigger.Request(), timeout_sec=30.0)
        finally:
            test_node.destroy_client(client)

    # ROS1 pre-set /can_motor_group/enable_controller=True on the parameter server here.
    # A ROS2 launch has no equivalent, so the stored-value reset is covered by
    # test_can_motor_group.py (test_stored_enable_does_not_start) instead.
    launch = start_launch('both', [])
    ready1 = call('/cm1/ROS2canIP_can_node_1/ready')
    ready2 = call('/cm2/ROS2canIP_can_node_2/ready')
    assert ready1.success and not ready2.success, (ready1, ready2)
    assert len(relay.samples) == 2 and all(not sample[2] for sample in relay.samples)
    group = Client('/can_motor_group', timeout=15)
    assert group.get_configuration(timeout=2)['enable_controller'] is False
    print('PASS: launch sends only disabled configuration; board ACK readiness is checked', flush=True)

    count = len(relay.samples)
    response = call('/can_motor_group/start')
    assert not response.success and len(relay.samples) == count, response
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is False
    assert len(relay.samples) == count
    print('PASS: rejected initial ACK prevents all enable requests', flush=True)

    assert call('/can_motor_group/prepare').success
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    assert relay.saw_concurrent_start
    starts = [sample for sample in relay.samples if sample[2]]
    assert len(starts) == 2 and {sample[0] for sample in starts} == {1, 2}
    print('PASS: both start commands finish before either start ACK is released', flush=True)
    count = len(relay.samples)
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    assert len(relay.samples) == count
    print('PASS: GUI refresh with unchanged enable value does not resend commands', flush=True)

    assert call('/can_motor_group/stop').success
    assert all(not sample[2] for sample in relay.samples[-2:])
    assert group.update_configuration({})['enable_controller'] is False
    print('PASS: group stop sends disable commands to both boards', flush=True)

    assert call('/can_motor_group/start').success
    assert group.update_configuration({})['enable_controller'] is True
    assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    assert all(not sample[2] for sample in relay.samples[-2:])
    print('PASS: start service updates checkbox; unchecking sends disable commands to both boards', flush=True)

    relay.drop_next_node2_start = True
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is False
    assert all(not sample[2] for sample in relay.samples[-2:])
    print('PASS: missing start ACK resets checkbox and disables both boards', flush=True)

    relay.drop_next_node2_start = True
    response = call('/can_motor_group/start')
    assert not response.success and 'disable attempt' in response.message, response
    assert all(not sample[2] for sample in relay.samples[-2:])
    assert group.update_configuration({})['enable_controller'] is False
    print('PASS: missing start ACK causes disable attempts for both boards', flush=True)

    assert call('/can_motor_group/start').success
    assert call('/can_motor_group/prepare').success
    assert group.update_configuration({})['enable_controller'] is False
    assert all(not sample[2] for sample in relay.samples[-2:])
    print('PASS: prepare service disables controllers and resets checkbox', flush=True)
    group.close()

    stop_process(launch)
    launch = None
    time.sleep(0.2)
    samples_before = len(relay.samples)
    launch = start_launch('single', ['motor2:=false'])
    assert call('/cm1/ROS2canIP_can_node_1/ready').success
    group = Client('/can_motor_group', timeout=15)
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    assert relay.samples[samples_before:] and all(sample[0] == 1 for sample in relay.samples[samples_before:])
    print('PASS: single-motor mode operates without CM2', flush=True)
    group.close()

    stop_process(launch)
    launch = None
    time.sleep(0.2)
    canip = FakeRelay(relay_mode=False)
    canip.reject_initial_node2 = False
    launch = start_launch('full_pipeline', [
        'start_receiver:=true', 'relay_port:={}'.format(free_port()),
        'canip_host:=127.0.0.1', 'canip_port:={}'.format(canip.port),
        'listen_port:={}'.format(free_port()), 'print_stats:=false',
    ])
    assert call('/cm1/ROS2canIP_can_node_1/ready').success
    assert call('/cm2/ROS2canIP_can_node_2/ready').success
    group = Client('/can_motor_group', timeout=15)
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    assert canip.saw_concurrent_start
    assert all(not sample[2] for sample in canip.samples[-2:])
    print('PASS: default launch starts real canIP2ROS relay and handles simulated CAN ACKs end to end', flush=True)
    group.close()

    # Added in the ROS2 port: ros2 launch stops every node at once, unlike roslaunch.
    before_stop = len(canip.samples)
    stop_process(launch)
    launch = None
    shutdown = canip.samples[before_stop:]
    assert {sample[0] for sample in shutdown} == {1, 2}, shutdown
    assert all(not any(sample[3]) for sample in shutdown), shutdown
    print('PASS: Ctrl-C delivers the all-zero idle command to both boards through the relay', flush=True)
    time.sleep(0.2)
    canip.saw_concurrent_start = False
    canip.require_both_start = True
    count_before = len(canip.samples)
    # ROS1 pre-set a stored group activity=4 here; see test_stored_activity_is_reset_without_commands.
    launch = start_launch('multi_activity', [
        'ctrl:=SMMultipleAct', 'start_receiver:=true', 'relay_port:={}'.format(free_port()),
        'canip_host:=127.0.0.1', 'canip_port:={}'.format(canip.port),
        'listen_port:={}'.format(free_port()), 'print_stats:=false',
    ])
    assert call('/cm1/ROS2canIP_can_node_1/ready').success
    assert call('/cm2/ROS2canIP_can_node_2/ready').success
    assert all(sample[3][0] == 9 and not sample[2] for sample in canip.samples[count_before:])
    motor1 = Client('/cm1/ROS2canIP_can_node_1', timeout=15)
    motor2 = Client('/cm2/ROS2canIP_can_node_2', timeout=15)
    group = Client('/can_motor_group', timeout=15)
    assert group.get_configuration(timeout=2)['activity'] == 0
    before = len(canip.samples)
    assert motor1.update_configuration({'phase': 1})['q_eq'] == 120
    assert len(canip.samples) == before
    motor1.update_configuration({'q_eq': 110.0})
    assert canip.samples[-1][3][9] == 110
    assert motor1.update_configuration({'activity': 1})['q_eq'] == 120
    wait_for_configuration(motor1, {'activity': 1, 'q_eq': 120})
    motor1.update_configuration({'q_eq': 115.0})
    assert len(canip.samples) == before + 1
    motor2.update_configuration({'activity': 1, 'phase': 1})
    motor2.update_configuration({'q_eq': 100.0})
    before = len(canip.samples)
    result = motor1.update_configuration({'save_name': 'cm1_test', 'save_now': True})
    assert result['save_now'] is False and len(canip.samples) == before
    import yaml
    snapshot = ROOT / 'home/cubemars_sm_params/node_1/cm1_test.yaml'
    document = yaml.safe_load(snapshot.read_text())
    assert document['can_node_id'] == 1
    assert document['activities']['OW']['phases']['Flexion']['q_eq'] == 110
    assert document['activities']['StairsAsc']['phases']['Flexion']['q_eq'] == 115
    print('PASS: real rqt interface keeps profiles independent, phase browsing sends no CAN, snapshot saves all activities', flush=True)
    before = len(canip.samples)
    assert group.update_configuration({'activity': 1})['activity'] == 1
    requests = canip.samples[before:]
    assert len(requests) == 2 and all(not sample[2] and sample[3][3] == 1 for sample in requests)
    assert {sample[0]: sample[3][9] for sample in requests} == {1: 115, 2: 100}
    assert motor1.update_configuration({})['requested_activity'] == 'StairsAsc'
    assert motor2.update_configuration({})['requested_activity'] == 'StairsAsc'
    print('PASS: one group activity sends two different motor-specific profiles without enabling either motor', flush=True)
    before = len(canip.samples)
    canip.drop_next_node2_activity = True
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is False
    assert all(not sample[2] for sample in canip.samples[before:])
    assert {sample[0] for sample in canip.samples[-2:]} == {1, 2}
    print('PASS: failed pre-start activity ACK prevents all enable requests and attempts group disable', flush=True)
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    assert canip.saw_concurrent_start
    assert all(sample[2] and sample[3][3] == 1 for sample in canip.samples[-2:])
    assert {sample[0]: sample[3][2] for sample in canip.samples[-2:]} == {1: 0, 2: 1}
    assert {sample[0]: sample[3][9] for sample in canip.samples[-2:]} == {1: 115, 2: 100}
    before = len(canip.samples)
    result = motor1.update_configuration({'load_name': 'cm1_test', 'load_now': True})
    assert result['load_now'] is False and len(canip.samples) == before
    result = motor1.update_configuration({'q_eq': 105.0})
    assert result['q_eq'] == 115 and len(canip.samples) == before
    motor1.update_configuration({'activity': 3})
    assert len(canip.samples) == before
    assert motor1.update_configuration({'requested_activity': 'RampDesc'})['requested_activity'] == 'StairsAsc'
    assert len(canip.samples) == before
    assert group.update_configuration({'activity': 3})['enable_controller'] is True
    assert all(sample[3][3] == 3 and sample[2] for sample in canip.samples[-2:])
    assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    print('PASS: editor dropdowns cannot change running activity; group switch sends both profiles without restart', flush=True)
    motor1.update_configuration({'activity': 1, 'q_eq': 115.0})
    motor1.update_configuration({'q_eq': 100.0})
    motor1.update_configuration({'load_name': 'cm1_test'})
    assert call('/cm1/ROS2canIP_can_node_1/LoadSMParams').success
    assert motor1.update_configuration({})['q_eq'] == 115
    assert canip.samples[-1][3][3] == 3 and not canip.samples[-1][2]
    motor2.update_configuration({'save_name': 'cm2_test'})
    assert call('/cm2/ROS2canIP_can_node_2/SaveSMParams').success
    snapshot2 = ROOT / 'home/cubemars_sm_params/node_2/cm2_test.yaml'
    assert yaml.safe_load(snapshot2.read_text())['can_node_id'] == 2
    assert not canip.samples[-1][2]
    print('PASS: private save/load services work without deadlock and keep per-motor YAML files separate', flush=True)
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    canip.drop_next_node2_activity = True
    result = group.update_configuration({'activity': 4})
    assert result['enable_controller'] is False and result['activity'] == 3
    assert {sample[0] for sample in canip.samples[-2:]} == {1, 2}
    assert all(not sample[2] for sample in canip.samples[-2:])
    assert group.update_configuration({'enable_controller': True})['enable_controller'] is True
    assert all(sample[2] and sample[3][3] == 3 for sample in canip.samples[-2:])
    assert group.update_configuration({'enable_controller': False, 'activity': 0})['activity'] == 0
    assert all(not sample[2] and sample[3][3] == 0 for sample in canip.samples[-2:])
    print('PASS: lost activity ACK disables both, reverts the group selection, and next start realigns profiles', flush=True)
    assert group.update_configuration({'starting_phase': 1, 'enable_controller': True})['enable_controller'] is True
    assert {sample[0]: sample[3][2] for sample in canip.samples[-2:]} == {1: 1, 2: 0}
    before = len(canip.samples)
    result = group.update_configuration({'activity': 5})
    assert result['activity'] == 0 and result['enable_controller'] is True
    result = group.update_configuration({'starting_phase': 0})
    assert result['starting_phase'] == 1 and len(canip.samples) == before
    print('PASS: locomotion starts with explicit opposite phases; live phase-policy changes send no commands', flush=True)
    result = group.update_configuration({'enable_controller': False, 'activity': 5, 'starting_phase': 0})
    assert result['activity'] == 5 and not result['enable_controller']
    assert all(not sample[2] and sample[3][3] == 5 for sample in canip.samples[-2:])
    motor1.update_configuration({'activity': 5, 'phase': 1})
    motor1.update_configuration({'q_eq': 130.0})
    motor2.update_configuration({'activity': 5, 'phase': 1})
    motor2.update_configuration({'q_eq': 125.0})
    for phase in (0, 1):
        result = group.update_configuration({'starting_phase': phase, 'enable_controller': True})
        assert result['enable_controller'] and result['activity'] == 5
        assert all(sample[2] and sample[3][2] == phase and sample[3][3] == 5 for sample in canip.samples[-2:])
        assert {sample[0]: sample[3][9] for sample in canip.samples[-2:]} == {1: 130, 2: 125}
        assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    motor1.update_configuration({'save_name': 'sts_test'})
    assert call('/cm1/ROS2canIP_can_node_1/SaveSMParams').success
    sts_snapshot = ROOT / 'home/cubemars_sm_params/node_1/sts_test.yaml'
    sts_document = yaml.safe_load(sts_snapshot.read_text())
    assert sts_document['schema_version'] == 2
    assert sts_document['activities']['StS']['phases']['Flexion']['q_eq'] == 130
    print('PASS: StS starts both phases together with independent motor parameters and six-activity snapshots', flush=True)
    motor1.close()
    motor2.close()
    group.close()
    stop_process(launch)
    launch = None
    time.sleep(0.2)
    count_before = len(canip.samples)
    launch = start_launch('multi_activity_single', [
        'ctrl:=SMMultipleAct', 'motor2:=false', 'start_receiver:=true',
        'relay_port:={}'.format(free_port()), 'canip_host:=127.0.0.1',
        'canip_port:={}'.format(canip.port), 'listen_port:={}'.format(free_port()),
        'print_stats:=false',
    ])
    assert call('/cm1/ROS2canIP_can_node_1/ready').success
    group = Client('/can_motor_group', timeout=15)
    result = group.update_configuration({'activity': 2, 'enable_controller': True})
    assert result['activity'] == 2 and result['enable_controller'] is True
    assert all(sample[0] == 1 for sample in canip.samples[count_before:])
    assert canip.samples[-1][2] and canip.samples[-1][3][3] == 2
    assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    result = group.update_configuration({'activity': 5, 'starting_phase': 1, 'enable_controller': True})
    assert result['enable_controller'] and canip.samples[-1][3][2:4] == (1, 5)
    assert all(sample[0] == 1 for sample in canip.samples[count_before:])
    assert group.update_configuration({'enable_controller': False})['enable_controller'] is False
    group.close()
    print('PASS: shared activity also works with only CM1 selected', flush=True)
finally:
    stop_process(launch)
    if rclpy is not None and rclpy.ok():
        rclpy.shutdown()
    if relay is not None:
        relay.close()
    if canip is not None:
        canip.close()
    for output in logs:
        output.close()

print('All isolated ROS launch/service/ACK checks passed. No physical CAN or motors were used.', flush=True)
