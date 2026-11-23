"""Bench stage 2 live check: prints one line per leg per second while you swing the arms.

Runs offline (no internet needed). Start it next to the phase node:
    /usr/bin/python3 -m kinematic_control.ros.stage2_monitor

Columns per leg, over the last second:
    angle    min..max thigh angle (deg, + flexion)
    phase    min..max phase (should sweep ~0 -> 1 while swinging)
    walk     fraction of samples with walking=1
    desired  min..max torque the controller WOULD send (Nm, + flexion)
    fault / dropped / sending   must stay 0 / small / 0 in stage 2
    -> OK / CHECK: a quick verdict
"""
import time
from collections import deque

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

LEGS = ("left", "right")
WINDOW_S = 1.0


class Stage2Monitor(Node):
    def __init__(self):
        super().__init__("stage2_monitor")
        self.buf = {leg: deque() for leg in LEGS}
        for leg in LEGS:
            self.create_subscription(Float64MultiArray, f"/phase_ctrl/{leg}",
                                     lambda m, leg=leg: self.on_msg(leg, m), 50)
        self.create_timer(WINDOW_S, self.report)
        self.t0 = time.time()
        print("waiting for /phase_ctrl/left and /phase_ctrl/right ... (is the phase node running?)")

    def on_msg(self, leg, msg):
        self.buf[leg].append((time.time(), list(msg.data)))

    def report(self):
        now = time.time()
        parts = [f"t={now - self.t0:5.0f}s"]
        for leg in LEGS:
            b = self.buf[leg]
            while b and b[0][0] < now - WINDOW_S:
                b.popleft()
            rows = [r for _, r in b if len(r) >= 8]
            if not rows:
                parts.append(f"{leg:5s}: no data")
                continue
            ang = [r[1] for r in rows]
            ph = [r[2] for r in rows]
            walk = sum(r[3] > 0.5 for r in rows) / len(rows)
            des = [r[4] for r in rows]
            fault = max(r[6] for r in rows)
            dropped = max(r[7] for r in rows)
            sending = max(r[8] for r in rows) if len(rows[0]) > 8 else 0.0
            moving = max(ang) - min(ang) > 10
            problems = []
            if fault:
                problems.append("FAULT")
            if sending:
                problems.append("SENDING!")
            if max(abs(d) for d in des) > 0.6:
                problems.append("torque>0.6")
            if moving and walk < 0.5:
                problems.append("not walking")
            verdict = "CHECK " + ",".join(problems) if problems else ("OK swinging" if moving else "OK still")
            parts.append(f"{leg:5s}: angle {min(ang):+5.1f}..{max(ang):+5.1f}  phase {min(ph):.2f}..{max(ph):.2f}  "
                         f"walk {walk:.0%}  desired {min(des):+.2f}..{max(des):+.2f}  "
                         f"f{fault:.0f} d{dropped:.0f} s{sending:.0f}  -> {verdict}")
        print(" | ".join(parts), flush=True)


def main():
    rclpy.init()
    node = Stage2Monitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
