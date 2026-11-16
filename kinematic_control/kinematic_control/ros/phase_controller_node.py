"""Live phase controller. COMPUTE-ONLY by default; with send on it streams torque to the boards.

    /robot_state_CubeMars_id{1,2}  ->  LegPipeline (frame -> phase portrait -> torque map -> safety)
                                   ->  /phase_ctrl/{left,right}
                                   ->  (send on) TorqueSender -> canIP2ROS relay -> board ctrlPhaseTorque

Published /phase_ctrl/<leg> (Float64MultiArray), one per board message:
    [0] board time      [1] hip angle (deg)     [2] gait phase 0..1 (NaN while warming up)
    [3] walking (0/1)   [4] desired torque (Nm, + = flexion, from the torque map)
    [5] safe torque     (after safety; what is sent when send is on)
    [6] fault (0/1)     [7] dropped ticks since the last message
    [8] sending (0/1)   (1 = a torque-on frame went to the board this tick)
"""
import math

import numpy as np

from kinematic_control.estimation.phase_portrait.phase_portrait import PhasePortrait, load_lookup
from kinematic_control.frames import LegFrame, check_board_frame
from kinematic_control.safety import LEG_MOTOR, make_leg_safety
from kinematic_control.torque_map import TorqueMap

DT = 0.02          # boards report at 50 Hz
STRIDE_M = 1.25    # fixed stride for the torque map (the phase portrait doesn't estimate it)
INCLINE = 0.0      # level walking for now


class LegPipeline:
    """Everything for one leg, no ROS. step() = one board message in, one row of numbers out."""

    def __init__(self, leg: str, tmap: TorqueMap, lookup, frame: LegFrame = None):
        self.leg = leg
        self.frame = frame or LegFrame(leg)
        check_board_frame(self.frame)                 # refuse to start with a wrong sign / zero
        self.tmap = tmap
        self.estimator = PhasePortrait(dt=DT, lookup=lookup)
        self.safety = make_leg_safety(leg, DT)
        self.safety.enable()
        self.last_time = None

    def dropped_ticks(self, board_time: float) -> int:
        """How many 50 Hz samples went missing before this one (WiFi/CAN drops)."""
        if self.last_time is None:
            n = 0
        else:
            n = max(0, round((board_time - self.last_time) / DT) - 1)
        self.last_time = board_time
        return n

    def step(self, board_time: float, board_angle: float, board_velocity: float) -> list:
        dropped = self.dropped_ticks(board_time)
        angle = float(self.frame.hip_angle(board_angle))
        velocity = float(self.frame.hip_velocity(board_velocity))

        phase = self.estimator.step(angle)
        walking = self.estimator.walking
        desired = self.tmap.command(phase, 0.0, STRIDE_M, INCLINE) if phase is not None else 0.0
        safe = self.safety.step(desired, walking, angle, velocity)

        return [board_time, angle, math.nan if phase is None else phase, float(walking),
                desired, safe, float(self.safety.fault is not None), float(dropped), 0.0]

    def reset(self) -> None:
        """Clear a latched fault and allow torque again (soft start applies)."""
        self.safety.reset()
        self.safety.enable()

    def restart_soft_start(self) -> None:
        """When sending turns on: the board was at 0 Nm, so ramp in from 0 instead of jumping
        to whatever the compute-only torque was."""
        self.safety.gain = 0.0
        self.safety.last_torque = 0.0


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from std_srvs.srv import SetBool, Trigger

    from kinematic_control.ros.torque_sender import TorqueSender

    class PhaseControllerNode(Node):
        def __init__(self):
            super().__init__("phase_controller")
            legs = self.declare_parameter("legs", "left,right").value.split(",")
            peak = float(self.declare_parameter("peak_Nm", 2.0).value)
            tmap, lookup = TorqueMap.load(peak_Nm=peak), load_lookup()

            self.pipes, self.pubs = {}, {}
            for leg in (l.strip() for l in legs):
                zero = float(self.declare_parameter(f"zero_{leg}", 0.0).value)
                self.pipes[leg] = LegPipeline(leg, tmap, lookup, LegFrame(leg, zero=zero))
                self.pubs[leg] = self.create_publisher(Float64MultiArray, f"/phase_ctrl/{leg}", 10)
                topic = f"/robot_state_CubeMars_id{LEG_MOTOR[leg]}"
                self.create_subscription(Float64MultiArray, topic,
                                         lambda msg, leg=leg: self.on_state(leg, msg), 10)
                self.get_logger().info(f"{leg}: {topic} -> /phase_ctrl/{leg}  (zero {zero:+.1f} deg)")

            self.create_service(Trigger, "~/reset", self.on_reset)
            self.create_service(SetBool, "~/send", self.on_send)
            self.faulted = set()
            relay_port = int(self.declare_parameter("relay_port", 12347).value)   # canIP2ROS command relay
            self.sender = TorqueSender(relay=("127.0.0.1", relay_port))
            self.sending = False
            self.set_sending(bool(self.declare_parameter("send", False).value))

        def set_sending(self, on: bool) -> None:
            if on and not self.sending:
                for pipe in self.pipes.values():
                    pipe.restart_soft_start()
            if not on and self.sending:
                for leg in self.pipes:
                    self.sender.send(LEG_MOTOR[leg], 0.0, False)
            self.sending = on
            peak = self.pipes[next(iter(self.pipes))].tmap.peak_Nm
            if on:
                self.get_logger().warning(f"SENDING torque to the boards (peak {peak} Nm, soft start 2 s)")
            else:
                self.get_logger().info(f"COMPUTE-ONLY (peak {peak} Nm): nothing is sent to the boards")

        def on_state(self, leg, msg):
            d = msg.data
            if len(d) < 3:
                return
            row = self.pipes[leg].step(d[0], d[1], d[2])
            fault = self.pipes[leg].safety.fault
            on = self.sending and fault is None
            if self.sending:                                  # one frame per state message
                self.sender.send(LEG_MOTOR[leg], row[5], on)  # fault -> torque-off frame
            row[8] = float(on)
            self.pubs[leg].publish(Float64MultiArray(data=row))
            if fault and leg not in self.faulted:       # log each fault once
                self.faulted.add(leg)
                self.get_logger().warning(f"{leg} safety fault: {fault} (torque held at 0; call ~/reset)")

        def on_send(self, request, response):
            self.set_sending(bool(request.data))
            response.success = True
            response.message = "sending torque" if self.sending else "compute-only (sent torque-off once)"
            return response

        def on_reset(self, request, response):
            for pipe in self.pipes.values():
                pipe.reset()
            self.faulted.clear()
            response.success, response.message = True, "faults cleared, soft start again"
            return response

    rclpy.init()
    node = PhaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sender.close([LEG_MOTOR[leg] for leg in node.pipes] if node.sending else [])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
