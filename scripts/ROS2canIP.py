#!/usr/bin/env python3

"""
ROS -> local relay -> canIP -> Yoctopus CAN command bridge.

This node mirrors the high-level command vector produced by ROS2tcp.py, but
encodes it as classic CAN frames wrapped in the CAPS/Yoctopus serial framing
expected by canIP.

Command CAN payload, 8 bytes:
    byte 0: command sequence number
    byte 1: signal index in the command vector
    byte 2: signal count in the command vector
    byte 3: reserved, currently 0
    byte 4..7: little-endian float32 value

CAN ID mapping:
    tx_base_id = can_cmd_base_id + can_target_node * can_id_stride
    can_id     = tx_base_id + signal_index

Defaults intentionally use a command base separate from ReporterCAN telemetry.
By default, command datagrams are sent to the localhost relay owned by
canIP2ROS.py so both ROS nodes can run while canIP sees only one UDP client.

ROS2 port of ROS2canIP.py from micropython_logger devel/CMbenchTop (ef24346).
CAN framing, ACK handling and the controller callbacks are unchanged;
dynamic_reconfigure is replaced by micropython_logger.reconfigure.ReconfigureServer.
"""

import os
import re
import socket
import sys
import struct
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from micropython_logger.srv import SetCANActivity

from micropython_logger import ctrl_configs
from micropython_logger.can_sm_profiles import CANSMProfiles, ACTIVITIES, STS_ACTIVITY, PHASES, PHASE_FIELDS, ACTIVITY_FIELDS
from micropython_logger.reconfigure import ReconfigureServer


def _private_arg_int(name, default):
    # ROS1 used _name:=value; ROS2 passes -p name:=value. Accept both.
    prefixes = ("_{}:=".format(name), "{}:=".format(name))
    for arg in sys.argv[1:]:
        for prefix in prefixes:
            if arg.startswith(prefix):
                return int(arg[len(prefix):], 0)
    return default


def _node_name_for_target(target_node):
    # ROS node names may contain letters, numbers, and underscores.
    target_text = re.sub(r"[^A-Za-z0-9_]", "_", str(target_node))
    return "ROS2canIP_can_node_{}".format(target_text)


def _load_ctrl_config(module_name):
    # "GuiTeleopConfig" -> ctrl_configs.GuiTeleop, generated from cfg/GuiTeleop.cfg.
    return getattr(ctrl_configs, module_name[:-len("Config")])


class ROS2canIP(Node):
    RELAY_ACK = b"CANIP_RELAY_OK"
    RELAY_ERROR = b"CANIP_RELAY_ERROR"
    RELAY_CAN_ACK_PREFIX = b"CANIP_RELAY_CAN_ACK"

    CTRL_CONFIGS = {
        "Teleop": (3, "GuiTeleopConfig", 20),
        "SMWalking": (4, "GuiSMWalking2StatesConfig", 20),
        "SMWalking4States": (4, "GuiSMWalkingConfig", 23),
        "SMWalking3States": (8, "GuiSMWalking3StatesConfig", 20),
        "SM3States": (8, "GuiSMWalking3StatesConfig", 20),
        "SMMultipleAct": (9, "GuiCubeMarsSMMultiActConfig", 20),
        "OscillatorForce": (5, "GuiOscillatorForceConfig", 20),
        "OscillatorPos": (6, "GuiOscillatorPosConfig", 20),
        "FF": (1, "GuiFFCalConfig", 20),
        "FB": (2, "GuiFBTranspConfig", 20),
        "TeleopCAN": (7, "GuiTeleopCANConfig", 20),
    }

    def __init__(self):
        initial_can_target_node = _private_arg_int("can_target_node", 4)
        node_name = _node_name_for_target(initial_can_target_node)
        super().__init__(node_name)

        self.robot = self._param("robot", "CubeMars")
        self.ctrl = self._resolve_ctrl_for_robot(
            self.robot,
            self._param("ctrl", "OscillatorPos"),
        )
        self.transport = self._param("transport", "udp").lower()
        self.canip_host = self._param("canip_host", "192.168.20.2")
        self.canip_port = self._param_int("canip_port", 10002)
        self.use_local_relay = self._param_bool("use_local_relay", True)
        self.relay_host = self._param("relay_host", "127.0.0.1")
        self.relay_port = self._param_int("relay_port", 12347)
        self.relay_ack_timeout_s = float(
            self._param("relay_ack_timeout_s", 0.5)
        )
        self.command_ack_enabled = self._param_bool("command_ack_enabled", True)
        self.command_ack_timeout_s = float(
            self._param("command_ack_timeout_s", 5.0)
        )
        self.command_ack_accept_modified_checksum = self._param_bool(
            "command_ack_accept_modified_checksum",
            True,
        )
        self.send_idle_on_shutdown = self._param_bool("send_idle_on_shutdown", True)
        self.start_disabled = self._param_bool("start_disabled", False)
        self.relay_startup_timeout_s = float(
            self._param("relay_startup_timeout_s", 0.0)
        )

        # CAN command ID mapping. Keep this separate from ReporterCAN telemetry IDs.
        self.can_cmd_base_id = self._param_int("can_cmd_base_id", 0x300)
        self.can_id_stride = self._param_int("can_id_stride", 32)
        self.can_target_node = self._param_int(
            "can_target_node",
            initial_can_target_node,
        )
        if self.can_target_node != initial_can_target_node and self.get_name() == node_name:
            self.get_logger().warning(
                "Node name was created for can_target_node={}, but the ROS parameter is {}. "
                "Pass -p can_target_node:=... on the command line if the node name should match.".format(
                    initial_can_target_node,
                    self.can_target_node,
                )
            )
        ctrl_config = self.CTRL_CONFIGS.get(self.ctrl)
        if ctrl_config is None:
            raise ValueError("Unknown ~ctrl '{}'. Options: {}".format(self.ctrl, sorted(self.CTRL_CONFIGS)))
        header, config_module_name, ctrl_msg_dim = ctrl_config
        if self.ctrl == "SMMultipleAct" and self.robot != "CubeMars":
            raise ValueError("The CAN SMMultipleAct controller is for CubeMars; the Fox controller uses ROS2tcp")
        config_params = _load_ctrl_config(config_module_name)
        self.ctrl_uses_starting_state = self.ctrl in (
            "SMWalking",
            "SM3States",
            "SMWalking3States",
            "SMWalking4States",
            "SMMultipleAct",
        )
        self.starting_state = float(
            self._param("starting_state", -1.0)
        )

        self.command_count = self._param_int("command_count", 1 + ctrl_msg_dim + 4)
        self.can_ack_id_offset = self._param_int(
            "can_ack_id_offset",
            self.can_id_stride - 1,
        )
        self.inter_frame_delay_s = float(self._param("inter_frame_delay_s", 0.002))
        self.repeat_count = int(self._param("repeat_count", 1))
        self.repeat_delay_s = float(self._param("repeat_delay_s", 0.02))
        self.udp_trigger_delay_s = float(self._param("udp_trigger_delay_s", 0.05))
        self.enable_teleop_subscriber = bool(self._param("enable_teleop_subscriber", True))

        if self.command_count < 1 or self.command_count > 255:
            raise ValueError("~command_count must be in [1, 255]")
        if self.can_id_stride < self.command_count:
            raise ValueError("~can_id_stride must be >= ~command_count")
        if self.command_ack_enabled and (
            self.can_ack_id_offset < self.command_count
            or self.can_ack_id_offset >= self.can_id_stride
        ):
            raise ValueError(
                "~can_ack_id_offset must be >= ~command_count and < ~can_id_stride"
            )
        if self.repeat_count < 1:
            self.repeat_count = 1
        if self.use_local_relay and self.transport != "udp":
            raise ValueError("The local canIP relay supports only UDP transport")

        self.tx_base_can_id = self.can_cmd_base_id + self.can_target_node * self.can_id_stride
        self.ack_can_id = self.tx_base_can_id + self.can_ack_id_offset
        if self.tx_base_can_id + self.command_count - 1 > 0x7FF:
            raise ValueError("Command CAN ID range exceeds 11-bit standard CAN IDs")
        if self.command_ack_enabled and self.ack_can_id > 0x7FF:
            raise ValueError("Command ACK CAN ID exceeds 11-bit standard CAN IDs")

        self.seq = 0
        self.sent_samples = 0
        self.sent_frames = 0
        self.last_send_time = None
        self.acks_received = 0
        self.ack_timeouts = 0
        self.ack_mismatches = 0
        self.ack_modified_checksums = 0
        self._queued_relay_acks = []
        self._closed = False
        self._idle_shutdown_sent = False
        self._command_lock = threading.RLock()
        self._command_result = threading.local()
        self._last_command_success = False
        self._initial_config = True
        self.header_msg = np.array([header], dtype=np.float32)
        self.ctrl_msg = np.zeros(ctrl_msg_dim, dtype=np.float32)
        self.robot_msg = np.zeros(4, dtype=np.float32)
        self.en = 0.0
        self.K = 0.0
        self.B = 0.0

        if self.ctrl == "SMMultipleAct":
            package_root = get_package_share_directory("micropython_logger")
            default_file = self._param(
                "sm_params_file", os.path.join(package_root, "cubemars_sm_params", "default.yaml")
            )
            saved_dir = self._param(
                "sm_saved_dir",
                os.path.join(os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")),
                             "cubemars_sm_params", "node_{}".format(self.can_target_node)),
            )
            self.sm_profiles = CANSMProfiles(default_file, saved_dir, self.can_target_node)
            if self.sm_profiles.migrated_legacy:
                self._logwarn("Loaded five-activity snapshot; added untuned StS bench defaults. Save creates a six-activity version-2 snapshot.")
            self._sm_activity = ACTIVITIES[0]
            self._sm_phase = PHASES[0]
            self._sm_requested_activity = ACTIVITIES[0]
            self._sm_enable_requested = False
            self._sm_file_result = threading.local()
            self._sm_control_request = threading.local()

        self._connect_socket()
        self._warn_if_id_range_contains_caps_som()

        print("ROS node", self.get_fully_qualified_name())
        print("Robot", self.robot)
        print("Ctrl", self.ctrl)
        if self.use_local_relay:
            print(
                "Command transport: local relay udp://{}:{} "
                "(canIP owner: canIP2ROS.py)".format(
                    self.relay_host,
                    self.relay_port,
                )
            )
        else:
            print(
                "Command transport: direct canIP {}://{}:{}".format(
                    self.transport,
                    self.canip_host,
                    self.canip_port,
                )
            )
        print(
            "Command CAN IDs: base=0x{:03X}, target_node={}, stride={}, tx_base=0x{:03X}, range=0x{:03X}..0x{:03X}".format(
                self.can_cmd_base_id,
                self.can_target_node,
                self.can_id_stride,
                self.tx_base_can_id,
                self.tx_base_can_id,
                self.tx_base_can_id + self.command_count - 1,
            )
        )
        print(
            "Command ACK: enabled={}, ack_id=0x{:03X}, timeout={:.3f}s, "
            "accept_modified_checksum={}".format(
                self.command_ack_enabled,
                self.ack_can_id,
                self.command_ack_timeout_s,
                self.command_ack_accept_modified_checksum,
            )
        )
        if self.ctrl_uses_starting_state:
            print(
                "SMWalking starting_state field: enabled=True, index=2, value={}".format(
                    self.starting_state
                )
            )
        print(
            "Shutdown idle command: enabled={}".format(
                self.send_idle_on_shutdown
            )
        )

        self.server = ReconfigureServer(self, config_params, self.callback)
        self.ready_service = self.create_service(Trigger, "~/ready", self.ready_callback)
        self.prepare_service = self.create_service(Trigger, "~/prepare", self.prepare_callback)
        self.start_service = self.create_service(Trigger, "~/start", self.start_callback)
        self.stop_service = self.create_service(Trigger, "~/stop", self.stop_callback)
        self.service_imu_cal = self.create_service(Trigger, "IMU_calibration", self.imu_cal_callback_)
        self.service_load_cell_cal = self.create_service(Trigger, "LoadCell_calibration", self.load_cell_cal_callback_)
        if self.ctrl == "SMMultipleAct":
            self.save_sm_service = self.create_service(Trigger, "~/SaveSMParams", self.save_sm_params_callback)
            self.load_sm_service = self.create_service(Trigger, "~/LoadSMParams", self.load_sm_params_callback)
            self.activity_service = self.create_service(SetCANActivity, "~/set_activity", self.set_activity_callback)

        if self.ctrl == "Teleop" and self.enable_teleop_subscriber:
            self.create_subscription(Float64MultiArray, "ctrl_command_" + self.robot, self.callback_teleop_subs, 1)

    def _param(self, name, default):
        # rospy parameters were untyped; launch files may pass ints, strings or floats.
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)
        ).value

    def _param_int(self, name, default):
        value = self._param(name, default)
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def _param_bool(self, name, default):
        value = self._param(name, default)
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    # rospy-style logging, so "%s" format arguments keep working.
    def _logwarn(self, message, *args):
        self.get_logger().warning(message % args if args else message)

    def _loginfo(self, message, *args):
        self.get_logger().info(message % args if args else message)

    def _logerr(self, message, *args):
        self.get_logger().error(message % args if args else message)

    @staticmethod
    def _resolve_ctrl_for_robot(robot, ctrl):
        return ctrl

    def _connect_socket(self):
        if self.use_local_relay:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(self.relay_ack_timeout_s)
            if self.relay_startup_timeout_s > 0:
                self._wait_for_local_relay()
            return

        if self.transport == "tcp":
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.settimeout(2.0)
            self.sock.connect((self.canip_host, self.canip_port))
            return

        if self.transport != "udp":
            raise ValueError("~transport must be 'udp' or 'tcp'")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.2)

    def _wait_for_local_relay(self):
        deadline = time.monotonic() + self.relay_startup_timeout_s
        previous_timeout = self.sock.gettimeout()
        try:
            while rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.sock.settimeout(min(self.relay_ack_timeout_s, remaining))
                try:
                    self.sock.sendto(b"hello", (self.relay_host, self.relay_port))
                    self._recv_until_relay_ok()
                    return
                except (OSError, RuntimeError):
                    time.sleep(min(0.05, max(deadline - time.monotonic(), 0.0)))
        finally:
            self.sock.settimeout(previous_timeout)
        raise RuntimeError("Local canIP relay did not become ready within {:.1f}s".format(
            self.relay_startup_timeout_s
        ))

    def _warn_if_id_range_contains_caps_som(self):
        ids_with_0x65 = [
            can_id for can_id in range(self.tx_base_can_id, self.tx_base_can_id + self.command_count)
            if (can_id & 0xFF) == 0x65
        ]
        if ids_with_0x65:
            self._logwarn(
                "Command CAN ID range contains low byte 0x65: {}. "
                "This is the CAPS serial start byte and may trigger the known Yoctopus parser issue.".format(
                    ["0x{:03X}".format(can_id) for can_id in ids_with_0x65]
                )
            )

    @staticmethod
    def _caps_checksum(hdr1, hdr2, payload):
        return (hdr1 + hdr2 + sum(payload)) & 0xFF

    @classmethod
    def _pack_caps_can_frame(cls, can_id, payload):
        if can_id < 0 or can_id > 0x7FF:
            raise ValueError("Only 11-bit standard CAN IDs are supported")
        if len(payload) > 8:
            raise ValueError("Classic CAN payload must be <= 8 bytes")

        hdr1 = ((len(payload) & 0x0F) << 3) | ((can_id >> 8) & 0x07)
        hdr2 = can_id & 0xFF
        checksum = cls._caps_checksum(hdr1, hdr2, payload)
        return bytes([0x65, hdr1, hdr2]) + bytes(payload) + bytes([checksum])

    def _build_command_vector(self):
        vec = np.zeros(self.command_count, dtype=np.float32)
        values = np.concatenate((self.header_msg, self.ctrl_msg, self.robot_msg)).astype(np.float32)
        n = min(len(values), self.command_count)
        vec[:n] = values[:n]
        return vec

    def _pack_command_sample(self, values):
        frames = []
        indices = list(range(len(values)))
        if values[0] == 9 and len(values) > 3:
            # canIP needs the activity before deciding the automatic start phase.
            indices[2], indices[3] = indices[3], indices[2]
        for signal_index in indices:
            value = values[signal_index]
            payload = bytearray(8)
            payload[0] = self.seq & 0xFF
            payload[1] = signal_index & 0xFF
            payload[2] = self.command_count & 0xFF
            payload[3] = 0
            struct.pack_into("<f", payload, 4, float(value))
            frames.append(self._pack_caps_can_frame(self.tx_base_can_id + signal_index, payload))
        return frames

    @staticmethod
    def _command_checksum(values):
        checksum = 2166136261
        for value in values:
            for byte_value in struct.pack("<f", float(value)):
                checksum ^= byte_value
                checksum = (checksum * 16777619) & 0xFFFFFFFF
        return checksum

    @classmethod
    def _parse_relay_can_ack(cls, response):
        if not response.startswith(cls.RELAY_CAN_ACK_PREFIX):
            return None

        try:
            text = response.decode("ascii", errors="replace")
        except UnicodeDecodeError:
            return None

        ack = {}
        for key, value in re.findall(r"([a-z_]+)=([^\s]+)", text):
            try:
                ack[key] = int(value, 0)
            except ValueError:
                ack[key] = value
        return ack

    def _consume_relay_response(self, response):
        ack = self._parse_relay_can_ack(response)
        if ack is None:
            return False
        self._queued_relay_acks.append(ack)
        return True

    def _recv_until_relay_ok(self):
        while True:
            response, _ = self.sock.recvfrom(512)
            if response == self.RELAY_ACK:
                return
            if response == self.RELAY_ERROR:
                raise RuntimeError("canIP2ROS relay rejected the command datagram")
            if self._consume_relay_response(response):
                continue
            raise RuntimeError(
                "Unexpected canIP2ROS relay response: {!r}".format(response)
            )

    def _send_bytes(self, data):
        if self.transport == "tcp":
            self.sock.sendall(data)
            return

        if self.use_local_relay:
            self.sock.sendto(data, (self.relay_host, self.relay_port))
            self._recv_until_relay_ok()
            return

        self.sock.sendto(data, (self.canip_host, self.canip_port))

    def _ack_matches(self, ack, expected_seq):
        return (
            ack.get("node") == self.can_target_node
            and ack.get("can_id") == self.ack_can_id
            and ack.get("seq") == expected_seq
        )

    def _validate_ack(self, ack, expected_seq, expected_checksum):
        status = ack.get("status")
        count = ack.get("count")
        checksum = ack.get("checksum")
        relay_expected_checksum = ack.get("expected_checksum")
        relay_checksum_match = ack.get("checksum_match")
        ctrl_header = ack.get("ctrl")
        checksum_is_int = isinstance(checksum, int)
        checksum_value = checksum if checksum_is_int else 0
        checksum_matches = checksum_is_int and checksum == expected_checksum
        checksum_modified_by_relay = (
            self.command_ack_accept_modified_checksum
            and relay_checksum_match == 0
            and relay_expected_checksum == expected_checksum
        )

        if count != self.command_count or (
            not checksum_matches and not checksum_modified_by_relay
        ):
            self.ack_mismatches += 1
            self._logwarn(
                "Ignoring ACK mismatch for node={} seq={}: status={} count={} "
                "checksum=0x{:08X} expected=0x{:08X}".format(
                    self.can_target_node,
                    expected_seq,
                    status,
                    count,
                    checksum_value,
                    expected_checksum,
                )
            )
            return None

        if status == 1:
            self.acks_received += 1
            if checksum_matches:
                print(
                    "ACK received: node={} id=0x{:03X} seq={} status=APPLIED "
                    "count={} ctrl={} checksum=0x{:08X}".format(
                        self.can_target_node,
                        self.ack_can_id,
                        expected_seq,
                        count,
                        ctrl_header,
                        checksum_value,
                    )
                )
            else:
                self.ack_modified_checksums += 1
                print(
                    "ACK received after high-level modification: node={} "
                    "id=0x{:03X} seq={} status=APPLIED count={} ctrl={} "
                    "checksum=0x{:08X} original_checksum=0x{:08X}".format(
                        self.can_target_node,
                        self.ack_can_id,
                        expected_seq,
                        count,
                        ctrl_header,
                        checksum_value,
                        expected_checksum,
                    )
                )
            return True

        self.ack_mismatches += 1
        self._logwarn(
            "ACK rejected for node={} seq={}: status={} count={} "
            "checksum=0x{:08X}".format(
                self.can_target_node,
                expected_seq,
                status,
                count,
                checksum_value,
            )
        )
        return False

    def _pop_matching_queued_ack(self, expected_seq):
        for idx, ack in enumerate(self._queued_relay_acks):
            if self._ack_matches(ack, expected_seq):
                return self._queued_relay_acks.pop(idx)
        return None

    def _wait_for_command_ack(self, expected_seq, expected_checksum):
        if not self.command_ack_enabled:
            return True
        if not self.use_local_relay:
            self._logwarn(
                "Command ACK wait is only implemented for the local relay path."
            )
            return True

        ack = self._pop_matching_queued_ack(expected_seq)
        if ack is not None:
            result = self._validate_ack(ack, expected_seq, expected_checksum)
            if result is not None:
                return result

        deadline = time.time() + self.command_ack_timeout_s
        previous_timeout = self.sock.gettimeout()
        try:
            while time.time() < deadline:
                self.sock.settimeout(max(deadline - time.time(), 0.001))
                try:
                    response, _ = self.sock.recvfrom(512)
                except socket.timeout:
                    break

                if response in (self.RELAY_ACK, self.RELAY_ERROR):
                    continue
                if not self._consume_relay_response(response):
                    continue

                ack = self._pop_matching_queued_ack(expected_seq)
                if ack is not None:
                    result = self._validate_ack(
                        ack,
                        expected_seq,
                        expected_checksum,
                    )
                    if result is not None:
                        return result
        finally:
            self.sock.settimeout(previous_timeout)

        self.ack_timeouts += 1
        self._logwarn(
            "No command ACK received for node={} seq={} id=0x{:03X} within {:.3f}s".format(
                self.can_target_node,
                expected_seq,
                self.ack_can_id,
                self.command_ack_timeout_s,
            )
        )
        return False

    def _trigger_udp_session(self):
        if self.transport != "udp" or self.use_local_relay:
            return

        # canIP consumes the first UDP datagram while establishing a client session.
        # Use its filtered keepalive token so the first command sample is preserved.
        self.sock.sendto(b"hello", (self.canip_host, self.canip_port))
        if self.udp_trigger_delay_s > 0:
            time.sleep(self.udp_trigger_delay_s)

    def send_command_sample(self, values=None, label="command"):
        # GUI callbacks, control services and calibration share one UDP socket.
        with self._command_lock:
            return self._send_command_sample_locked(values, label)

    def _send_command_sample_locked(self, values, label):
        sent_seq = self.seq & 0xFF
        self._command_result.success = False
        self._last_command_success = False
        try:
            self._trigger_udp_session()
            if values is None:
                values = self._build_command_vector()
            else:
                values = np.asarray(values, dtype=np.float32)
            if len(values) != self.command_count:
                raise RuntimeError(
                    "Command vector length {} does not match command_count {}".format(
                        len(values),
                        self.command_count,
                    )
                )
            expected_checksum = self._command_checksum(values)
            frames = self._pack_command_sample(values)

            for repeat_idx in range(self.repeat_count):
                if self.inter_frame_delay_s <= 0:
                    self._send_bytes(b"".join(frames))
                else:
                    for frame in frames:
                        self._send_bytes(frame)
                        time.sleep(self.inter_frame_delay_s)

                if repeat_idx + 1 < self.repeat_count:
                    time.sleep(self.repeat_delay_s)
        except (OSError, RuntimeError) as error:
            # A partially forwarded sample may already exist on the board.
            # Give the next attempt a fresh sequence instead of mixing samples.
            self.seq = (self.seq + 1) & 0xFF
            if self.use_local_relay:
                self._logerr(
                    "Command relay failed: {}. Start canIP2ROS.py and verify "
                    "relay port {}.".format(error, self.relay_port)
                )
            else:
                self._logerr("Direct canIP command send failed: {}".format(error))
            return False

        self.sent_samples += 1
        self.sent_frames += len(frames) * self.repeat_count
        self.last_send_time = time.time()
        print(
            "Sent {} sample seq={} frames={} range=0x{:03X}..0x{:03X}".format(
                label,
                self.seq,
                len(frames) * self.repeat_count,
                self.tx_base_can_id,
                self.tx_base_can_id + len(frames) - 1,
            )
        )
        ack_ok = self._wait_for_command_ack(sent_seq, expected_checksum)
        self._command_result.success = ack_ok
        self._last_command_success = ack_ok
        self.seq = (self.seq + 1) & 0xFF
        return ack_ok

    def send_idle_command(self, label="idle"):
        values = np.zeros(self.command_count, dtype=np.float32)
        return self.send_command_sample(values=values, label=label)

    def callback(self, config, level):
        with self._command_lock:
            return self._config_callback(config, level)

    def _config_callback(self, config, level):
        initial = self._initial_config
        if self._initial_config:
            self._initial_config = False
            if self.start_disabled:
                config["enable_controller"] = False
        if self.ctrl == "SMMultipleAct":
            return self._multi_activity_callback(config, initial)
        print("---------------------")
        config_index = 0
        ctrl_index = 0
        for k in config.keys():
            if k == "groups":
                continue
            if k in ("starting_state", "start_state", "sm_starting_state"):
                self.starting_state = float(config[k])
                print(k, config[k])
                continue

            print(k, config[k])

            if self.ctrl_uses_starting_state and config_index == 1:
                if ctrl_index < len(self.ctrl_msg):
                    self.ctrl_msg[ctrl_index] = self.starting_state
                ctrl_index += 1

            if ctrl_index < len(self.ctrl_msg):
                self.ctrl_msg[ctrl_index] = float(config[k])
            ctrl_index += 1
            config_index += 1

        if self.ctrl_uses_starting_state:
            print("starting_state", self.starting_state)

        if self.ctrl == "Teleop":
            self.en = float(config.get("enable_controller", self.en))
            self.K = float(config.get("K_stiffness", self.K))
            self.B = float(config.get("B_damping", self.B))

        print("---------------------")
        self.send_command_sample()
        return config

    def _multi_activity_callback(self, config, initial):
        self._command_result.success = False
        requested = getattr(self._sm_control_request, "activity", None)
        requested_start = getattr(self._sm_control_request, "starting_state", None)
        if requested is not None and self._sm_enable_requested and (
            (requested == STS_ACTIVITY) != (ACTIVITIES.index(self._sm_requested_activity) == STS_ACTIVITY)
            or (requested_start is not None and requested_start != self.starting_state)
        ):
            config["requested_activity"] = self._sm_requested_activity
            self._logwarn("Stop the controller before changing starting phase or switching into/out of StS")
            return config
        activity = ACTIVITIES[int(config["activity"])]
        phase = PHASES[int(config["phase"])]
        selection_changed = activity != self._sm_activity or phase != self._sm_phase
        file_action = bool(config["save_now"] or config["load_now"])
        loaded = False
        profile_updated = False
        self._sm_file_result.result = (False, "No save/load operation requested")
        # This GUI field is status only. Requests arrive via the typed service.
        config["requested_activity"] = self._sm_requested_activity

        if initial:
            # A saved GUI enable or save/load checkbox must not act at startup.
            config["enable_controller"] = False
            config["save_now"] = config["load_now"] = False
            self.sm_profiles.display(config, activity, phase)
        elif selection_changed:
            # A dropdown change still contains the outgoing row's values.
            # They have already been cached on each successful edit.
            self.sm_profiles.display(config, activity, phase)
        else:
            current = dict(config)
            self.sm_profiles.display(current, activity, phase)
            changed = any(config[field] != current[field] for field in PHASE_FIELDS + ACTIVITY_FIELDS)
            if changed and self._sm_enable_requested and config["enable_controller"]:
                self._logwarn("Stop the CubeMars controller before editing activity parameters")
                self.sm_profiles.display(config, activity, phase)
                config["save_now"] = config["load_now"] = False
                return config
            if changed:
                try:
                    self.sm_profiles.update(config, activity, phase)
                    profile_updated = True
                except ValueError as error:
                    self._logwarn("CubeMars profile rejected: %s", error)
                    self.sm_profiles.display(config, activity, phase)
                    # Still honor a disable request even if its edit was invalid.
                    if config["enable_controller"] == self._sm_enable_requested:
                        config["save_now"] = config["load_now"] = False
                        return config

        self._sm_activity, self._sm_phase = activity, phase

        if not initial and file_action:
            loading = bool(config["load_now"])
            try:
                if config["save_now"] and loading:
                    raise ValueError("Request save or load, not both together")
                if loading:
                    if self._sm_enable_requested:
                        raise ValueError("Stop the controller before loading parameters")
                    filename = self.sm_profiles.load(config["load_name"])
                    if self.sm_profiles.migrated_legacy:
                        self._logwarn("Five-activity snapshot loaded; StS uses untuned bench defaults")
                    self.sm_profiles.display(config, activity, phase)
                    config["enable_controller"] = False
                    loaded = True
                else:
                    filename = self.sm_profiles.save(config["save_name"])
                self._sm_file_result.result = (True, "{} {}".format("Loaded" if loading else "Saved", filename))
                self._loginfo(self._sm_file_result.result[1])
            except (OSError, ValueError) as error:
                self._sm_file_result.result = (False, str(error))
                self._logwarn("CubeMars save/load failed: %s", error)
            finally:
                config["save_now"] = config["load_now"] = False

        if requested is not None:
            self._sm_requested_activity = ACTIVITIES[requested]
            self.starting_state = float(requested_start)
        config["requested_activity"] = self._sm_requested_activity
        enable_changed = bool(config["enable_controller"]) != self._sm_enable_requested
        send_profile = profile_updated and activity == self._sm_requested_activity
        force = getattr(self._sm_control_request, "force", False)
        # Both dropdowns are editors. Only the requested activity goes on CAN.
        if not (initial or force or enable_changed or loaded or send_profile):
            return config

        self.ctrl_msg[:] = 0.0
        self.ctrl_msg[0] = float(config["enable_controller"])
        self.ctrl_msg[1] = self.starting_state
        self.ctrl_msg[2] = float(ACTIVITIES.index(self._sm_requested_activity))
        self.ctrl_msg[3:15] = self.sm_profiles.wire_values(self._sm_requested_activity)
        self._sm_enable_requested = bool(config["enable_controller"])
        self._loginfo("CubeMars requested_activity=%s editing=%s/%s enable=%s starting_state=%s",
                      self._sm_requested_activity, activity, phase, self._sm_enable_requested, self.starting_state)
        confirmed = self.send_command_sample()
        if loaded and not confirmed:
            self._sm_file_result.result = (False, "Loaded profile locally, but the board command was not confirmed")
        return config

    def set_activity_callback(self, req, response=None):
        if req.activity < 0 or req.activity >= len(ACTIVITIES):
            return SetCANActivity.Response(success=False, message="Activity must be in [0,5]")
        if req.starting_state not in (-1, 0, 1):
            return SetCANActivity.Response(success=False, message="Starting state must be -1 (auto), 0 (Extension), or 1 (Flexion)")
        if self._closed:
            return SetCANActivity.Response(success=False, message="Command node is closing")
        self._sm_control_request.activity = req.activity
        self._sm_control_request.starting_state = req.starting_state
        self._sm_control_request.force = True
        try:
            self.server.update_configuration({})
            success = self._command_result.success
            return SetCANActivity.Response(
                success=success,
                message="node={} activity={} starting_state={}: {}".format(
                    self.can_target_node, ACTIVITIES[req.activity], req.starting_state,
                    "profile ACK received (may be pending local transition)" if success else "profile not confirmed",
                ),
            )
        except Exception as error:
            return SetCANActivity.Response(success=False, message=str(error))
        finally:
            del self._sm_control_request.activity
            del self._sm_control_request.starting_state
            del self._sm_control_request.force

    def _sm_file_service(self, loading):
        name_field = "load_name" if loading else "save_name"
        name = self.get_parameter(name_field).value
        self.server.update_configuration({name_field: name, "load_now" if loading else "save_now": True})
        success, message = self._sm_file_result.result
        return Trigger.Response(success=success, message=message)

    def save_sm_params_callback(self, req, response=None):
        return self._sm_file_service(False)

    def load_sm_params_callback(self, req, response=None):
        return self._sm_file_service(True)

    def ready_callback(self, req, response=None):
        with self._command_lock:
            if not self.command_ack_enabled or not self.use_local_relay:
                return Trigger.Response(success=False, message="Board ACKs through the local relay are required")
            ready = not self._closed and self._last_command_success
            return Trigger.Response(
                success=ready,
                message="node={}: {}".format(
                    self.can_target_node,
                    "latest command ACK received" if ready else "latest command has no valid board ACK; prepare or update parameters again",
                ),
            )

    def _set_enabled(self, enabled, action):
        if self._closed:
            return Trigger.Response(success=False, message="Command node is closing")
        try:
            # The reconfigure callback runs in this thread. Keep its result
            # thread-local so a concurrent GUI update cannot replace this ACK.
            self._command_result.success = False
            if self.ctrl == "SMMultipleAct":
                self._sm_control_request.force = True
            self.server.update_configuration({"enable_controller": enabled})
            success = self._command_result.success
        except Exception as error:
            return Trigger.Response(success=False, message=str(error))
        finally:
            if self.ctrl == "SMMultipleAct":
                del self._sm_control_request.force
        return Trigger.Response(
            success=success,
            message="node={} {}: {}".format(
                self.can_target_node,
                action,
                "command applied" if success else "command not confirmed; check the board and relay",
            ),
        )

    def prepare_callback(self, req, response=None):
        return self._set_enabled(False, "prepare")

    def start_callback(self, req, response=None):
        return self._set_enabled(True, "start")

    def stop_callback(self, req, response=None):
        return self._set_enabled(False, "stop")

    def callback_teleop_subs(self, data):
        with self._command_lock:
            self._teleop_callback(data)

    def _teleop_callback(self, data):
        values = list(data.data)
        if values:
            values[0] = self.en
        if len(values) > 3:
            values[3] = self.K
        if len(values) > 4:
            values[4] = self.B

        n = min(len(values), len(self.ctrl_msg))
        if n > 0:
            self.ctrl_msg[:n] = np.array(values[:n], dtype=np.float32)
        self.send_command_sample()

    def imu_cal_callback_(self, req, response=None):
        with self._command_lock:
            self._loginfo("IMU Cal called!")
            self.robot_msg[0] = 1.0
            try:
                success = self.send_command_sample()
            finally:
                self.robot_msg[0] = 0.0
        msg = "IMU cal command sent over CAN." if success else "IMU cal CAN command failed."
        return Trigger.Response(success=success, message=msg)

    def load_cell_cal_callback_(self, req, response=None):
        with self._command_lock:
            self._loginfo("LOAD cell cal called!")
            self.robot_msg[1] = 1.0
            try:
                success = self.send_command_sample()
            finally:
                self.robot_msg[1] = 0.0
        msg = "Load cell cal command sent over CAN." if success else "Load cell cal CAN command failed."
        return Trigger.Response(success=success, message=msg)

    def close(self):
        with self._command_lock:
            self._close_locked()

    def _close_locked(self):
        if self._closed:
            return
        self._closed = True

        if self.send_idle_on_shutdown and not self._idle_shutdown_sent:
            self._idle_shutdown_sent = True
            print("Sending idle command before shutdown.")
            self.send_idle_command(label="shutdown idle")

        try:
            self.sock.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ROS2canIP()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Disable the board before the process exits; this only needs the socket.
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
