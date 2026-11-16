"""TorqueSender tests: a fake relay socket receives the frames and decodes them the way
canIP2ROS (CAPS checks) and the board's ctrlPhaseTorque (payload) do."""
import socket
import struct

import pytest

from kinematic_control.ros.torque_sender import TorqueSender, caps_frame


def parse_caps(data):
    """Same checks as canIP2ROS._extract_caps_frames_from_datagram."""
    assert data[0] == 0x65
    hdr1, hdr2 = data[1], data[2]
    n = (hdr1 >> 3) & 0x0F
    assert (hdr1 >> 7) & 1 == 0 and n == 8 and len(data) == 4 + n
    payload = data[3:3 + n]
    assert data[3 + n] == (hdr1 + hdr2 + sum(payload)) & 0xFF
    return ((hdr1 & 0x07) << 8) | hdr2, payload


def board_decode(payload):
    """Same as ctrlPhaseTorque._poll_can."""
    return payload[0], bool(payload[1] & 1), struct.unpack("<f", bytes(payload[4:8]))[0]


@pytest.fixture
def relay():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(1.0)
    yield s
    s.close()


def test_frames_reach_relay_and_decode(relay):
    tx = TorqueSender(relay=relay.getsockname())
    tx.send(2, 1.25, True)
    tx.send(2, -0.5, True)
    tx.send(1, 3.0, False)                       # off -> torque forced to 0

    sid, p = parse_caps(relay.recv(64)); assert sid == 0x402 and board_decode(p) == (0, True, 1.25)
    sid, p = parse_caps(relay.recv(64)); assert sid == 0x402 and board_decode(p) == (1, True, -0.5)
    sid, p = parse_caps(relay.recv(64)); assert sid == 0x401 and board_decode(p) == (0, False, 0.0)
    assert p[2] == 0                             # relay must not treat it as a command sample
    assert tx.sent == 3 and tx.errors == 0


def test_relay_error_reply_is_counted(relay):
    tx = TorqueSender(relay=relay.getsockname())
    tx.send(1, 0.0, True)
    _, addr = relay.recvfrom(64)
    relay.sendto(b"CANIP_RELAY_OK", addr)
    relay.sendto(b"CANIP_RELAY_ERROR", addr)
    import time; time.sleep(0.05)
    tx.send(1, 0.0, True)                        # drains replies
    assert tx.errors == 1


def test_seq_wraps():
    tx = TorqueSender(relay=("127.0.0.1", 9))    # discard port; nothing listens
    tx.seq[1] = 255
    tx.send(1, 0.0, True)
    assert tx.seq[1] == 0


def test_ids_avoid_caps_start_byte():
    for node in (1, 2):
        assert (0x400 + node) & 0xFF != 0x65     # known Yoctopus parser issue
    with pytest.raises(ValueError):
        caps_frame(0x800, b"\0" * 8)
