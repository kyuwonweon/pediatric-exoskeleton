"""Stream torque frames to the boards' ctrlPhaseTorque controller."""
import socket
import struct

TORQUE_CAN_BASE_ID = 0x400
RELAY = ("127.0.0.1", 12347)
RELAY_OK = b"CANIP_RELAY_OK"


def caps_frame(can_id: int, payload: bytes) -> bytes:
    if not 0 <= can_id <= 0x7FF or len(payload) > 8:
        raise ValueError("classic 11-bit CAN frame only")
    hdr1 = ((len(payload) & 0x0F) << 3) | ((can_id >> 8) & 0x07)
    hdr2 = can_id & 0xFF
    return bytes([0x65, hdr1, hdr2]) + bytes(payload) + bytes([(hdr1 + hdr2 + sum(payload)) & 0xFF])


def torque_payload(seq: int, torque_Nm: float, on: bool) -> bytes:
    p = bytearray(8)
    p[0] = seq & 0xFF
    p[1] = 0x01 if on else 0x00
    struct.pack_into("<f", p, 4, float(torque_Nm))
    return bytes(p)


class TorqueSender:
    def __init__(self, relay=RELAY, base_id: int = TORQUE_CAN_BASE_ID):
        self.relay = relay
        self.base_id = base_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.seq = {}
        self.sent = 0
        self.errors = 0          # send failures + relay error replies

    def send(self, node_id: int, torque_Nm: float, on: bool) -> None:
        seq = self.seq.get(node_id, 0)
        self.seq[node_id] = (seq + 1) & 0xFF
        frame = caps_frame(self.base_id + node_id, torque_payload(seq, torque_Nm if on else 0.0, on))
        try:
            self.sock.sendto(frame, self.relay)
            self.sent += 1
        except OSError:
            self.errors += 1
        self._drain_replies()

    def _drain_replies(self) -> None:
        while True:
            try:
                reply, _ = self.sock.recvfrom(256)
            except (BlockingIOError, OSError):
                return
            if not reply.startswith(RELAY_OK):
                self.errors += 1

    def close(self, node_ids=()) -> None:
        """Send torque-off to each board once, then close (the board would time out anyway)."""
        for n in node_ids:
            self.send(n, 0.0, False)
        self.sock.close()
