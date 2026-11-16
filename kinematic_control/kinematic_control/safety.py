"""Safety layer: sits between the torque map and the motor.

Whatever the estimator or torque map asks for, the motor never gets a command that could hurt.

Input  (every tick): desired torque (Nm, + = flexion), walking (bool), hip angle (deg) and
                     hip velocity (deg/s), both in dataset frame (+ = flexion)
Output (every tick): safe torque (Nm, + = flexion). frames.LegFrame.joint_torque() turns it into
                     the board's joint frame; the board's set_tau_offset() then applies its own
                     joint_direction, so no extra per-motor sign is applied here.

Layers inside step(), in order:
    1 faults  -> latch OFF: torque 0 at once, stays 0 until reset()
    2 gate    -> 0 unless enabled and walking
    3 soft start -> gain ramps 0->1 over soft_start_s once allowed; back to 0 when not
    4 angle limits -> near the end of the range, drop torque that pushes further out
    5 cap     -> |torque| <= max_torque
    6 rate limit -> torque changes at most max_rate Nm/s

A fault skips 3-6 and cuts to 0 immediately (the exo is backdrivable: zero torque is safe).
A normal stop (not walking, or disable()) just sets the target to 0; the rate limit then
brings the torque down smoothly (2 Nm -> 0 in 0.1 s), so no separate fade-out is needed.
"""
from dataclasses import dataclass, replace

import numpy as np


@dataclass
class Limits:
    """Every safety number in one place. Units: Nm, s, deg, deg/s."""
    max_torque: float = 2.0         # hard ceiling (unworn friction ~0.7 Nm; motor could do ~8)
    max_rate: float = 20.0          # Nm/s; 0 -> 2 Nm takes 0.1 s
    soft_start_s: float = 2.0       # ramp-in once walking (and after every stop)
    flex_limit: float = 70.0        # beyond this, no torque pushing further into flexion
    ext_limit: float = -20.0        # beyond this, no torque pushing further into extension
    fault_angle_margin: float = 30.0   # this far past a limit = fault (sensor or strap problem)
    fault_jump: float = 15.0        # angle change in one tick that can't be physical
    fault_velocity: float = 600.0   # deg/s


LEG_MOTOR = {"left": 1, "right": 2}  # id1 = LEFT, id2 = RIGHT (bench, 2026-10-26)

# Per-leg limits (hip angle, anatomical frame: 0 = standing upright, + = flexion).
LEG_LIMITS = {
    "left": Limits(flex_limit=50, ext_limit=-20),
    "right": Limits(flex_limit=60, ext_limit=-15),
}


class Safety:
    def __init__(self, limits: Limits = None, dt: float = 0.02):
        self.limits = limits if limits is not None else Limits()
        self.dt = dt
        self.enabled = False      # you turn it on (enable()); off at start
        self.fault = None         # None, or a short text saying why it latched off
        self.gain = 0.0           # 3 soft start: 0..1
        self.last_torque = 0.0    # 6 rate limit: what we sent last tick
        self.last_angle = None    # 1 faults: to spot impossible jumps

    # ---- on / off ----------------------------------------------------------------------------
    def enable(self) -> None:
        """Allow torque (still gated by walking, soft-started). Refused while a fault is latched."""
        if self.fault is None:
            self.enabled = True

    def disable(self) -> None:
        """Stop torque: target 0, rate limit brings it down (like a normal stop)."""
        self.enabled = False

    def reset(self) -> None:
        """Clear a latched fault. Leaves enabled = False: you must enable() again on purpose."""
        self.fault = None
        self.enabled = False
        self.gain = 0.0
        self.last_torque = 0.0
        self.last_angle = None

    # ---- 1. Faults ---------------------------------------------------------------------------
    def check_faults(self, torque: float, angle: float, velocity: float):
        """Return a reason string if something is impossible or broken, else None:
        NaN/inf input, angle jump > fault_jump in one tick, |velocity| > fault_velocity,
        angle more than fault_angle_margin past a limit."""
        L = self.limits
        if not (np.isfinite(torque) and np.isfinite(angle) and np.isfinite(velocity)):
            return "non-finite input"
        if self.last_angle is not None and abs(angle - self.last_angle) > L.fault_jump:
            return f"angle jumped {angle - self.last_angle:+.1f} deg in one tick"
        if abs(velocity) > L.fault_velocity:
            return f"velocity {velocity:.0f} deg/s"
        if angle > L.flex_limit + L.fault_angle_margin or angle < L.ext_limit - L.fault_angle_margin:
            return f"angle {angle:.1f} deg far outside the range"
        return None

    # ---- 2. Gate -----------------------------------------------------------------------------
    def gate(self, torque: float, walking: bool) -> float:
        """The torque we'd like if allowed: torque if enabled and walking, else 0."""
        return torque if (self.enabled and walking) else 0.0

    # ---- 3. Soft start -----------------------------------------------------------------------
    def soft_start(self, allowed: bool) -> float:
        """If allowed, raise self.gain toward 1 by dt / soft_start_s per tick; if not, set it to 0
        (so every restart ramps in again). Returns the gain."""
        if allowed:
            self.gain = min(1.0, self.gain + self.dt / self.limits.soft_start_s)
        else:
            self.gain = 0.0
        return self.gain

    # ---- 4. Angle limits ---------------------------------------------------------------------
    def angle_limit(self, torque: float, angle: float) -> float:
        """Past flex_limit, drop positive (flexion) torque; past ext_limit, drop negative.
        Torque pushing back toward the middle passes."""
        if angle >= self.limits.flex_limit and torque > 0:
            return 0.0
        if angle <= self.limits.ext_limit and torque < 0:
            return 0.0
        return torque

    # ---- 5. Cap ------------------------------------------------------------------------------
    def cap(self, torque: float) -> float:
        """Clip to +-max_torque."""
        m = self.limits.max_torque
        return max(-m, min(m, torque))

    # ---- 6. Rate limit -----------------------------------------------------------------------
    def rate_limit(self, torque: float) -> float:
        """Move from last_torque toward torque by at most max_rate * dt."""
        d = self.limits.max_rate * self.dt
        return max(self.last_torque - d, min(self.last_torque + d, torque))

    # ---- One tick ----------------------------------------------------------------------------
    def step(self, torque: float, walking: bool, angle: float, velocity: float) -> float:
        """One tick: desired torque in, safe torque out (Nm, + = flexion)."""
        reason = self.check_faults(torque, angle, velocity)
        if reason is not None and self.fault is None:
            self.fault = reason
            self.enabled = False
        self.last_angle = angle
        if self.fault is not None:
            self.gain, self.last_torque = 0.0, 0.0
            return 0.0                                  # instant cut, skips fade and rate limit
        allowed = self.enabled and walking
        tau = self.gate(torque, walking) * self.soft_start(allowed)
        tau = self.cap(self.angle_limit(tau, angle))
        tau = self.rate_limit(tau)
        self.last_torque = tau
        return tau


def make_leg_safety(leg: str, dt: float = 0.02) -> Safety:
    """A Safety for "left" or "right", with its own copy of that leg's limits
    (a copy, so changing one Safety's limits never changes LEG_LIMITS or another Safety)."""
    return Safety(replace(LEG_LIMITS[leg]), dt)
