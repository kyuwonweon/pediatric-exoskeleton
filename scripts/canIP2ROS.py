#!/usr/bin/env python3
"""Receive CAPS-framed CAN telemetry from canIP and publish ROS topics.

ROS2 port of canIP2ROS.py from micropython_logger devel/CMbenchTop (ef24346).
Socket, CAPS and ACK-relay handling is unchanged; only the ROS calls differ.
"""

import ast
import select
import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float64MultiArray


class canIP2ROS(Node):
    """Receive CAPS-framed CAN telemetry from canIP and publish ROS topics."""

    RELAY_ACK = b"CANIP_RELAY_OK"
    RELAY_ERROR = b"CANIP_RELAY_ERROR"
    RELAY_CAN_ACK_PREFIX = b"CANIP_RELAY_CAN_ACK"
    TIMESTAMP_MAGIC = b"CBMT"
    TIMESTAMP_HEADER_LEN = 16
    TIMESTAMP_HEADER_VERSION = 1
    MAX_CANIP_DATAGRAMS_PER_LOOP = 64
    MAX_RELAY_DATAGRAMS_PER_LOOP = 16

    def __init__(self):
        super().__init__("canIP2ROS")
        print("Ros Init success")

        self.robot = self._get_param("robot", "CubeMars")

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

        self.canip_host = self._get_param("canip_host", "192.168.20.2")
        self.canip_port = self._get_int_param("canip_port", 10002)
        listen_host = self._get_param("listen_host", "0.0.0.0")
        listen_port = self._get_int_param("listen_port", 12346)
        self.command_relay_host = self._get_param(
            "command_relay_host", "127.0.0.1"
        )
        self.command_relay_port = self._get_int_param(
            "command_relay_port", 12347
        )
        self.canip_keepalive_sec = float(
            self._get_param("canip_keepalive_sec", 1.0)
        )
        self.canip_max_datagrams_per_loop = self._get_int_param(
            "canip_max_datagrams_per_loop",
            self.MAX_CANIP_DATAGRAMS_PER_LOOP,
        )
        self.canip_gap_warn_s = float(self._get_param("canip_gap_warn_s", 0.2))
        self.print_stats = self._get_bool_param("print_stats", True)
        self.shutdown_relay_grace_s = float(
            self._get_param("shutdown_relay_grace_s", 2.0)
        )
        self.print_timing_warnings = self._get_bool_param(
            "print_timing_warnings",
            False,
        )
        self.append_canip_timestamp = self._get_bool_param(
            "append_canip_timestamp",
            True,
        )
        # When a sample is missing one or more of its 27 CAN frames, publishing
        # it fills the gaps with 0.0 (looks like a joint snapping to 0 deg).
        # Default: drop the incomplete sample instead, leaving a gap in the
        # topic stream (publish rate becomes non-uniform on frame loss).
        self.publish_partial_samples = self._get_bool_param(
            "publish_partial_samples",
            False,
        )
        if self.canip_max_datagrams_per_loop < 1:
            self.canip_max_datagrams_per_loop = 1

        self.can_base_id = self._get_int_param("can_base_id", 0x500)
        self.can_cmd_base_id = self._get_int_param("can_cmd_base_id", 0x300)
        self.can_id_stride = self._get_int_param("can_id_stride", 32)
        self.can_ack_id_offset = self._get_int_param(
            "can_ack_id_offset",
            self.can_id_stride - 1,
        )
        self.command_ack_sender_ttl_s = float(
            self._get_param("command_ack_sender_ttl_s", 2.0)
        )
        self.command_ack_accept_modified_checksum = self._get_bool_param(
            "command_ack_accept_modified_checksum",
            True,
        )
        if self.can_id_stride <= 0:
            raise ValueError("can_id_stride must be greater than zero")
        if self.can_ack_id_offset < 0 or self.can_ack_id_offset >= self.can_id_stride:
            raise ValueError("can_ack_id_offset must be inside the CAN ID stride")

        raw_nodes = self._get_param("can_nodes", [2, 3])
        self.can_target_nodes = self._parse_node_list(raw_nodes)
        if not self.can_target_nodes:
            raise ValueError("can_nodes must contain at least one non-negative node")

        self.pub_robot_by_node = {}
        self.pub_ctrl_by_node = {}
        for node_id in self.can_target_nodes:
            topic_robot = "robot_state_{}_id{}".format(self.robot, node_id)
            topic_ctrl = "ctrl_state_{}_id{}".format(self.robot, node_id)
            self.pub_robot_by_node[node_id] = self.create_publisher(
                Float64MultiArray,
                topic_robot,
                10,
            )
            self.pub_ctrl_by_node[node_id] = self.create_publisher(
                Float64MultiArray,
                topic_ctrl,
                10,
            )
            print(
                "ROS topic mapping node_id={}: {}, {}".format(
                    node_id, topic_robot, topic_ctrl
                )
            )

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.setblocking(False)
        self.s.bind((listen_host, listen_port))

        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.setblocking(False)
        self.command_socket.bind(
            (self.command_relay_host, self.command_relay_port)
        )

        self.last_canip_ping = 0.0
        self._caps_buffer = bytearray()
        self._node_samples = {}
        self._command_ack_waiters = {}
        self.node_last_index = {}
        self.node_lost_packets = {}

        self.last_time = None
        self.max_udp_gap_s = 0.0
        self.udp_gap_warnings = 0
        self.packet_count = 0
        self.total_time = 0.0
        self.last_pub_time = time.time()
        self.pub_count = 0
        self.receiving_addr = ""
        self.can_frames_seen = 0
        self.can_samples_completed = 0
        self.can_samples_partial = 0
        self.can_samples_dropped = 0
        self.command_datagrams_forwarded = 0
        self.command_bytes_forwarded = 0
        self.command_datagrams_rejected = 0
        self.command_acks_seen = 0
        self.command_acks_forwarded = 0
        self.command_acks_without_waiter = 0
        self.command_acks_checksum_mismatch = 0
        self.command_acks_modified_forwarded = 0
        self.timestamped_datagrams_seen = 0
        self.timestamped_samples_published = 0
        self.timestamp_header_errors = 0

        local_host, local_port = self.s.getsockname()
        print("Listening (yoctopus_canip) on {}:{}".format(local_host, local_port))
        print(
            "Forwarding source canIP is {}:{}".format(
                self.canip_host, self.canip_port
            )
        )
        print(
            "canIP UDP drain: max_datagrams_per_loop={}, stats={}, timing_warnings={}".format(
                self.canip_max_datagrams_per_loop,
                self.print_stats,
                self.print_timing_warnings,
            )
        )
        print(
            "canIP timestamp header: auto-detect, append_to_ros={}".format(
                self.append_canip_timestamp
            )
        )
        print(
            "Command relay listening on udp://{}:{}".format(
                self.command_relay_host,
                self.command_relay_port,
            )
        )
        print(
            "CAN mapping: base_id=0x{:03X}, stride={}, nodes={}".format(
                self.can_base_id,
                self.can_id_stride,
                self.can_target_nodes,
            )
        )
        print(
            "Command ACK mapping: cmd_base=0x{:03X}, ack_offset={}, "
            "ack_id=node_slot+offset".format(
                self.can_cmd_base_id,
                self.can_ack_id_offset,
            )
        )
        print(
            "Command ACK modified checksum forwarding: enabled={}".format(
                self.command_ack_accept_modified_checksum
            )
        )
        self._send_canip_trigger(force=True)

    def _get_param(self, name, default):
        # rospy parameters were untyped; launch files may pass ints, strings or lists.
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)
        ).value

    @staticmethod
    def _parse_int(value):
        if isinstance(value, str):
            return int(value, 0)
        return int(value)

    def _get_int_param(self, name, default):
        return self._parse_int(self._get_param(name, default))

    @staticmethod
    def _parse_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _get_bool_param(self, name, default):
        return self._parse_bool(self._get_param(name, default))

    def _parse_node_list(self, raw_nodes):
        items = raw_nodes
        if isinstance(raw_nodes, str):
            text = raw_nodes.strip()
            try:
                items = ast.literal_eval(text)
            except (SyntaxError, ValueError):
                items = [
                    item.strip()
                    for item in text.strip("[](){}").split(",")
                    if item.strip()
                ]

        if not isinstance(items, (list, tuple, set)):
            items = [items]

        nodes = []
        for item in items:
            try:
                nodes.append(self._parse_int(item))
            except (TypeError, ValueError):
                self.get_logger().warning("Ignoring invalid CAN node value: {}".format(item))

        return sorted(set(node for node in nodes if node >= 0))

    def _send_canip_trigger(self, force=False):
        now = time.time()
        if not force and now - self.last_canip_ping < self.canip_keepalive_sec:
            return

        try:
            self.s.sendto(b"hello", (self.canip_host, self.canip_port))
            self.last_canip_ping = now
        except OSError as error:
            self.get_logger().warning("Failed to ping canIP: {}".format(error))

    @staticmethod
    def _calc_checksum(hdr1, hdr2, payload):
        return (hdr1 + hdr2 + sum(payload)) & 0xFF

    @classmethod
    def _is_valid_caps_stream(cls, data):
        """Return True when data is one or more complete classic-CAN CAPS frames."""
        return bool(cls._extract_caps_frames_from_datagram(data))

    @classmethod
    def _extract_caps_frames_from_datagram(cls, data):
        """Extract valid frames from a single complete CAPS datagram."""
        frames = []
        offset = 0

        while offset < len(data):
            if data[offset] != ord("e") or offset + 4 > len(data):
                return []

            hdr1 = data[offset + 1]
            hdr2 = data[offset + 2]
            msg_type = (hdr1 >> 7) & 0x01
            data_len = (hdr1 >> 3) & 0x0F
            frame_len = 4 + data_len

            if msg_type != 0 or data_len > 8:
                return []
            if offset + frame_len > len(data):
                return []

            payload = data[offset + 3:offset + 3 + data_len]
            checksum = data[offset + 3 + data_len]
            if checksum != cls._calc_checksum(hdr1, hdr2, payload):
                return []

            sid = ((hdr1 & 0x07) << 8) | hdr2
            frames.append((sid, bytes(payload)))
            offset += frame_len

        return frames

    def _reply_to_command_sender(self, response, sender):
        try:
            self.command_socket.sendto(response, sender)
        except OSError as error:
            self.get_logger().warning("Failed to reply to command relay client: {}".format(error))

    def _node_slot_from_command_sid(self, sid):
        raw_id = int(sid) - self.can_cmd_base_id
        if raw_id < 0:
            return None, None
        return raw_id // self.can_id_stride, raw_id % self.can_id_stride

    @staticmethod
    def _checksum_from_command_payloads(payloads):
        checksum = 2166136261
        for payload in payloads:
            for byte_value in payload[4:8]:
                checksum ^= byte_value
                checksum = (checksum * 16777619) & 0xFFFFFFFF
        return checksum

    def _remember_command_ack_waiters(self, frames, sender):
        now = time.time()
        for sid, payload in frames:
            if len(payload) < 8:
                continue

            node_id, slot_index = self._node_slot_from_command_sid(sid)
            if node_id is None:
                continue

            seq = payload[0]
            signal_index = payload[1]
            signal_count = payload[2]
            if (
                signal_count < 1
                or signal_index >= signal_count
                or slot_index != signal_index
                or slot_index >= self.can_ack_id_offset
            ):
                continue

            key = (node_id, seq)
            state = self._command_ack_waiters.get(key)
            if state is None or state.get("signal_count") != signal_count:
                state = {
                    "sender": sender,
                    "updated": now,
                    "signal_count": signal_count,
                    "payloads": [None] * signal_count,
                    "received": bytearray(signal_count),
                    "received_count": 0,
                    "checksum": None,
                }
                self._command_ack_waiters[key] = state
            else:
                state["sender"] = sender
                state["updated"] = now

            if not state["received"][signal_index]:
                state["received"][signal_index] = 1
                state["received_count"] += 1
            state["payloads"][signal_index] = payload

            if state["received_count"] == state["signal_count"]:
                state["checksum"] = self._checksum_from_command_payloads(
                    state["payloads"]
                )

    def _cleanup_command_ack_waiters(self):
        if not self._command_ack_waiters:
            return

        now = time.time()
        expired = [
            key for key, state in self._command_ack_waiters.items()
            if now - state["updated"] > self.command_ack_sender_ttl_s
        ]
        for key in expired:
            self._command_ack_waiters.pop(key, None)

    def _forward_command_datagram(self, data, sender):
        # Accept the old trigger as a harmless local relay health check.
        if data == b"hello":
            self._reply_to_command_sender(self.RELAY_ACK, sender)
            return True

        frames = self._extract_caps_frames_from_datagram(data)
        if not frames:
            self.command_datagrams_rejected += 1
            self._reply_to_command_sender(self.RELAY_ERROR, sender)
            self.get_logger().warning(
                "Rejected malformed local CAPS command datagram from {}".format(
                    sender
                )
            )
            return False

        try:
            self.s.sendto(data, (self.canip_host, self.canip_port))
        except OSError as error:
            self.command_datagrams_rejected += 1
            self._reply_to_command_sender(self.RELAY_ERROR, sender)
            self.get_logger().warning("Failed to forward command to canIP: {}".format(error))
            return False

        self.command_datagrams_forwarded += 1
        self.command_bytes_forwarded += len(data)
        self._remember_command_ack_waiters(frames, sender)
        self._reply_to_command_sender(self.RELAY_ACK, sender)
        return True

    def _drain_command_relay(self):
        drained = 0
        while drained < self.MAX_RELAY_DATAGRAMS_PER_LOOP:
            try:
                data, sender = self.command_socket.recvfrom(65535)
            except BlockingIOError:
                break

            if data:
                self._forward_command_datagram(data, sender)
            drained += 1

        self._cleanup_command_ack_waiters()
        return drained

    def _strip_canip_timestamp_header(self, data):
        if not data.startswith(self.TIMESTAMP_MAGIC):
            return data, None

        if len(data) < self.TIMESTAMP_HEADER_LEN:
            self.timestamp_header_errors += 1
            self.get_logger().warning(
                "Dropping short canIP timestamp packet len={}".format(len(data))
            )
            return b"", None

        version = data[4]
        header_len = data[5]
        if (
            version != self.TIMESTAMP_HEADER_VERSION
            or header_len < self.TIMESTAMP_HEADER_LEN
            or header_len > len(data)
        ):
            self.timestamp_header_errors += 1
            self.get_logger().warning(
                "Dropping invalid canIP timestamp header: version={} "
                "header_len={} packet_len={}".format(
                    version,
                    header_len,
                    len(data),
                )
            )
            return b"", None

        timestamp_us = struct.unpack("<Q", data[8:16])[0]
        self.timestamped_datagrams_seen += 1
        return data[header_len:], timestamp_us

    def _handle_canip_datagram(self, data, sender):
        if not data:
            return

        now = time.time()
        if self.last_time is not None:
            gap = now - self.last_time
            self.total_time += gap
            if gap > self.max_udp_gap_s:
                self.max_udp_gap_s = gap
            if (
                self.print_timing_warnings
                and self.canip_gap_warn_s > 0.0
                and gap > self.canip_gap_warn_s
            ):
                self.udp_gap_warnings += 1
                print(
                    "[Timing] raw canIP UDP gap: {:.3f}s before packet len={} "
                    "from {}".format(gap, len(data), sender)
                )

        self.last_time = now
        self.packet_count += 1
        self.receiving_addr = sender
        data, canip_timestamp_us = self._strip_canip_timestamp_header(data)
        if data:
            self._process_canip_packet(data, canip_timestamp_us)

    def _drain_canip_socket(self):
        drained = 0
        while drained < self.canip_max_datagrams_per_loop:
            try:
                data, sender = self.s.recvfrom(65535)
            except BlockingIOError:
                break

            self._handle_canip_datagram(data, sender)
            drained += 1

        return drained

    def _extract_caps_frames(self, chunk):
        """Extract valid standard-CAN CAPS frames from a byte stream."""
        frames = []
        self._caps_buffer.extend(chunk)
        offset = 0

        while offset < len(self._caps_buffer):
            if self._caps_buffer[offset] != ord("e"):
                offset += 1
                continue

            if offset + 4 > len(self._caps_buffer):
                break

            hdr1 = self._caps_buffer[offset + 1]
            hdr2 = self._caps_buffer[offset + 2]
            msg_type = (hdr1 >> 7) & 0x01
            data_len = (hdr1 >> 3) & 0x0F
            frame_len = 4 + data_len

            if offset + frame_len > len(self._caps_buffer):
                break

            payload = bytes(
                self._caps_buffer[offset + 3:offset + 3 + data_len]
            )
            checksum = self._caps_buffer[offset + 3 + data_len]
            if (
                msg_type == 0
                and checksum == self._calc_checksum(hdr1, hdr2, payload)
            ):
                sid = ((hdr1 & 0x07) << 8) | hdr2
                frames.append((sid, payload))

            offset += frame_len

        if offset:
            del self._caps_buffer[:offset]

        return frames

    def _node_id_from_sid(self, sid):
        raw_id = int(sid) - self.can_base_id
        if raw_id < 0:
            return None
        return raw_id // self.can_id_stride

    def _handle_command_ack_frame(self, sid, payload):
        node_id, slot_index = self._node_slot_from_command_sid(sid)
        if node_id is None or slot_index != self.can_ack_id_offset:
            return False
        if len(payload) < 8:
            return True

        seq = payload[0]
        status = payload[1]
        received_count = payload[2]
        ctrl_header = payload[3]
        checksum = struct.unpack("<I", payload[4:8])[0]
        self.command_acks_seen += 1

        status_text = "APPLIED" if status == 1 else "REJECTED"
        print(
            "Command ACK received: node={} id=0x{:03X} seq={} status={} "
            "count={} ctrl={} checksum=0x{:08X}".format(
                node_id,
                sid,
                seq,
                status_text,
                received_count,
                ctrl_header,
                checksum,
            )
        )

        waiter = self._command_ack_waiters.pop((node_id, seq), None)
        if waiter is None:
            self.command_acks_without_waiter += 1
            return True

        expected_checksum = waiter.get("checksum")
        if expected_checksum is None:
            # We saw an ACK before reconstructing the locally forwarded sample.
            # Keep waiting for a later ACK instead of consuming this waiter.
            self._command_ack_waiters[(node_id, seq)] = waiter
            self.command_acks_without_waiter += 1
            return True
        checksum_match = checksum == expected_checksum
        if not checksum_match:
            self.command_acks_checksum_mismatch += 1
            if not self.command_ack_accept_modified_checksum:
                self._command_ack_waiters[(node_id, seq)] = waiter
                print(
                    "Ignoring stale/mismatched command ACK: node={} seq={} "
                    "checksum=0x{:08X} expected=0x{:08X}".format(
                        node_id,
                        seq,
                        checksum,
                        expected_checksum,
                    )
                )
                return True

            self.command_acks_modified_forwarded += 1
            print(
                "Forwarding command ACK with modified checksum: node={} seq={} "
                "checksum=0x{:08X} expected=0x{:08X}".format(
                    node_id,
                    seq,
                    checksum,
                    expected_checksum,
                )
            )

        response = (
            "{} node={} can_id=0x{:03X} seq={} status={} count={} "
            "ctrl={} checksum=0x{:08X} expected_checksum=0x{:08X} checksum_match={}"
        ).format(
            self.RELAY_CAN_ACK_PREFIX.decode("ascii"),
            node_id,
            sid,
            seq,
            status,
            received_count,
            ctrl_header,
            checksum,
            expected_checksum,
            1 if checksum_match else 0,
        ).encode("ascii")
        self._reply_to_command_sender(response, waiter["sender"])
        self.command_acks_forwarded += 1
        return True

    def _get_node_sample_state(self, node_id):
        state = self._node_samples.get(node_id)
        if state is None:
            state = {
                "seq": None,
                "expected_count": self.len_msg,
                "published": False,
                "canip_timestamp_us": None,
                "values": [0.0] * self.len_msg,
                "valid": [False] * self.len_msg,
            }
            self._node_samples[node_id] = state
        return state

    def _reset_node_sample(self, state, seq, signal_count):
        state["seq"] = seq
        state["expected_count"] = min(
            max(int(signal_count), 1),
            self.len_msg,
        )
        state["published"] = False
        state["canip_timestamp_us"] = None
        state["values"] = [0.0] * self.len_msg
        state["valid"] = [False] * self.len_msg

    def _finalize_previous_sample_if_needed(self, node_id, state):
        if state["seq"] is None or state["published"]:
            return
        if not any(state["valid"]):
            return
        if not self.publish_partial_samples:
            # Incomplete sample (>=1 CAN frame lost). Drop it rather than
            # publishing zeros for the missing signals; the topic just skips
            # a cycle.
            self.can_samples_dropped += 1
            state["published"] = True
            return
        self._publish_vector(
            node_id,
            state["values"],
            state["seq"],
            state.get("canip_timestamp_us"),
        )
        self.can_samples_partial += 1
        state["published"] = True

    def _process_canip_packet(self, data, canip_timestamp_us=None):
        for sid, payload in self._extract_caps_frames(data):
            if len(payload) < 8:
                continue

            if self._handle_command_ack_frame(sid, payload):
                continue

            node_id = self._node_id_from_sid(sid)
            if node_id not in self.can_target_nodes:
                continue

            seq = payload[0]
            signal_index = payload[1]
            signal_count = payload[2]
            if signal_index >= self.len_msg:
                continue

            signal_value = struct.unpack("<f", payload[4:8])[0]
            self.can_frames_seen += 1
            state = self._get_node_sample_state(node_id)

            if state["seq"] is None or seq != state["seq"]:
                self._finalize_previous_sample_if_needed(node_id, state)
                self._reset_node_sample(state, seq, signal_count)

            if (
                canip_timestamp_us is not None
                and (
                    signal_index == 0
                    or state.get("canip_timestamp_us") is None
                )
            ):
                state["canip_timestamp_us"] = canip_timestamp_us

            state["values"][signal_index] = float(signal_value)
            state["valid"][signal_index] = True

            if state["published"]:
                continue

            expected_count = state["expected_count"]
            if all(state["valid"][:expected_count]):
                self._publish_vector(
                    node_id,
                    state["values"],
                    seq,
                    state.get("canip_timestamp_us"),
                )
                self.can_samples_completed += 1
                state["published"] = True

    def _publish_vector(self, node_id, values, sequence, canip_timestamp_us=None):
        if not rclpy.ok():
            # Shutdown relay window: commands still flow, telemetry has no ROS context.
            return
        start_index = 0
        msg_header = [
            float(value)
            for value in values[
                start_index:start_index + self.dim_reporter_msg
            ]
        ]
        start_index += self.dim_reporter_msg

        start_index += self.dim_ctrl_fact_msg
        msg_ctrl = [
            float(value)
            for value in values[start_index:start_index + self.dim_ctrl_msg]
        ]
        start_index += self.dim_ctrl_msg

        start_index += self.dim_dec_msg
        msg_robot = [
            float(value)
            for value in values[start_index:start_index + self.dim_robot_msg]
        ]

        timestamp_suffix = []
        if self.append_canip_timestamp and canip_timestamp_us is not None:
            timestamp_suffix = [float(canip_timestamp_us) / 1000000.0]
            self.timestamped_samples_published += 1

        self.pub_robot_by_node[node_id].publish(
            Float64MultiArray(data=msg_header + msg_robot + timestamp_suffix)
        )
        self.pub_ctrl_by_node[node_id].publish(
            Float64MultiArray(data=msg_header + msg_ctrl + timestamp_suffix)
        )
        self.pub_count += 1
        self._update_loss_stats(node_id, sequence)

    def _update_loss_stats(self, node_id, sequence):
        current = int(sequence)
        previous = self.node_last_index.get(node_id)
        self.node_lost_packets.setdefault(node_id, 0)

        if previous is not None:
            expected = (previous + 1) % 256
            if current != expected:
                self.node_lost_packets[node_id] += (current - expected) % 256

        self.node_last_index[node_id] = current

    def _log_stats(self):
        if self.print_stats and (
            self.packet_count > 0 or self.command_datagrams_forwarded > 0
        ):
            rx_frequency = (
                self.packet_count / self.total_time
                if self.total_time > 0
                else 0.0
            )
            elapsed = max(time.time() - self.last_pub_time, 1e-6)
            pub_frequency = self.pub_count / elapsed
            lost = ", ".join(
                "id{}:{}".format(
                    node_id,
                    self.node_lost_packets.get(node_id, 0),
                )
                for node_id in self.can_target_nodes
            )
            print(
                "[Stats] Rec from addr: {} | Rx freq: {:.1f} Hz | "
                "ROS pub freq: {:.1f} Hz | CAN frames: {} | "
                "Samples full: {} partial: {} dropped: {} | Lost by node: {} | "
                "Max UDP gap: {:.3f}s warnings={} | "
                "Cmd relay: {} datagrams/{} bytes rejected={}".format(
                    self.receiving_addr,
                    rx_frequency,
                    pub_frequency,
                    self.can_frames_seen,
                    self.can_samples_completed,
                    self.can_samples_partial,
                    self.can_samples_dropped,
                    lost,
                    self.max_udp_gap_s,
                    self.udp_gap_warnings,
                    self.command_datagrams_forwarded,
                    self.command_bytes_forwarded,
                    self.command_datagrams_rejected,
                )
            )
            print(
                "[Stats] Command ACKs: seen={} forwarded={} no_waiter={} "
                "checksum_mismatch={} modified_forwarded={}".format(
                    self.command_acks_seen,
                    self.command_acks_forwarded,
                    self.command_acks_without_waiter,
                    self.command_acks_checksum_mismatch,
                    self.command_acks_modified_forwarded,
                )
            )
            print(
                "[Stats] canIP timestamps: packets={} samples={} "
                "header_errors={}".format(
                    self.timestamped_datagrams_seen,
                    self.timestamped_samples_published,
                    self.timestamp_header_errors,
                )
            )

        self.packet_count = 0
        self.total_time = 0.0
        self.max_udp_gap_s = 0.0
        self.udp_gap_warnings = 0
        self.pub_count = 0
        self.can_frames_seen = 0
        self.can_samples_completed = 0
        self.can_samples_partial = 0
        self.can_samples_dropped = 0
        self.command_datagrams_forwarded = 0
        self.command_bytes_forwarded = 0
        self.command_datagrams_rejected = 0
        self.command_acks_seen = 0
        self.command_acks_forwarded = 0
        self.command_acks_without_waiter = 0
        self.command_acks_checksum_mismatch = 0
        self.command_acks_modified_forwarded = 0
        self.timestamped_datagrams_seen = 0
        self.timestamped_samples_published = 0
        self.timestamp_header_errors = 0
        self.last_pub_time = time.time()

    def main_loop(self):
        while rclpy.ok():
            self._send_canip_trigger()

            try:
                readable, _, _ = select.select(
                    (self.s, self.command_socket),
                    (),
                    (),
                    0.02,
                )

                if self.command_socket in readable:
                    self._drain_command_relay()

                if self.s in readable:
                    self._drain_canip_socket()

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

    def relay_during_shutdown(self):
        """Keep forwarding commands while the ROS2canIP nodes send their shutdown idle.

        roslaunch stopped this node last, after the command nodes. ros2 launch
        signals every process at once, so without this window the idle (disable)
        commands find no relay and never reach the boards.
        """
        if self.shutdown_relay_grace_s <= 0:
            return
        print("Relaying shutdown commands for {:.1f}s".format(self.shutdown_relay_grace_s))
        deadline = time.monotonic() + self.shutdown_relay_grace_s
        while time.monotonic() < deadline:
            try:
                readable, _, _ = select.select((self.s, self.command_socket), (), (), 0.02)
                if self.command_socket in readable:
                    self._drain_command_relay()
                if self.s in readable:
                    self._drain_canip_socket()
            except Exception as error:
                print("Shutdown relay error: {}".format(error))
                time.sleep(0.05)

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass
        try:
            self.command_socket.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    bridge = canIP2ROS()
    # The socket loop owns the main thread; the executor only serves parameter requests.
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spinner = threading.Thread(target=_spin, args=(executor,))
    spinner.start()
    try:
        bridge.main_loop()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.relay_during_shutdown()
        bridge.close()
        # Stop and join the spin thread before teardown; a live thread aborts the process.
        executor.shutdown()
        spinner.join(timeout=2.0)
        bridge.destroy_node()
        rclpy.try_shutdown()
        print("Connection ended, waiting for new client...")


def _spin(executor):
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
