"""Board joint angle -> anatomical hip angle, per leg.

Since 2026-11-12 the boards already report the anatomical frame (params.json "Hip convention"):
    board = wrap(joint_direction * (servo - joint_zero_deg))
    0 = thigh vertical in aligned upright standing, + = flexion (thigh forward), deg.
Velocity uses the same direction, and robot.set_tau_offset() applies joint_direction to torque,
so a + torque we send is flexion on both legs. Nothing on our side flips signs any more.

What is left here is a small per-session correction and a check:

    hip = wrap(sign * (board - zero))

    zero: the board angle while you stand still and upright, measured each session. It should be
          within a few degrees of 0 (straps, posture); a big value means params.json is off.
    sign: should come out +1 on both legs; -1 means joint_direction in params.json is wrong.

The defaults (zero=0, sign=+1) trust the board as is.
This is what safety.LEG_LIMITS and the estimators expect.
"""
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np


def wrap(angle):
    """Any angle (deg) -> [-180, 180), like the board's _wrap_angle."""
    return (np.asarray(angle) + 180.0) % 360.0 - 180.0


@dataclass
class LegFrame:
    leg: str            # "left" or "right"
    zero: float = 0.0   # board angle while standing upright (deg); ~0 with the new board frame
    sign: int = +1      # +1: board angle increases with flexion (expected); -1: it decreases

    def hip_angle(self, board_angle):
        """Board joint angle (deg) -> anatomical hip angle (deg, 0 = standing, + = flexion)."""
        return wrap(self.sign * (np.asarray(board_angle) - self.zero))

    def hip_velocity(self, board_velocity):
        """Board joint velocity (deg/s) -> anatomical (deg/s). Only the sign changes."""
        return self.sign * np.asarray(board_velocity)

    def joint_torque(self, hip_torque):
        """Anatomical torque (Nm, + = flexion) -> torque to send to the board.
        The board applies joint_direction itself, so with sign=+1 this is unchanged."""
        return self.sign * np.asarray(hip_torque)


# ---- Calibration -------------------------------------------------------------------------------
def measure_zero(board_angles, max_spread_deg: float = 1.0) -> float:
    """Standing-still samples (e.g. 5 s at 50 Hz) -> the zero for that leg.
    Uses the median (ignores a few bad samples). Refuses if the leg moved: a zero taken while
    moving would shift every angle and every safety limit."""
    a = np.asarray(board_angles, dtype=float)
    if a.size < 10 or not np.isfinite(a).all():
        raise ValueError("need at least 10 finite samples to measure the zero")
    centre = float(np.median(a))
    spread = float(np.max(np.abs(wrap(a - centre))))  # wrap: works even near +-180
    if spread > max_spread_deg:
        raise ValueError(f"leg moved during zeroing (spread {spread:.1f} deg > {max_spread_deg}); stand still and redo")
    return centre


def measure_sign(board_angles_during_flex, min_change_deg: float = 10.0) -> int:
    """Samples while you slowly lift the thigh forward (flex) from standing -> +1 or -1.
    +1 if the board angle went up, -1 if it went down. Refuses if it barely moved."""
    a = np.asarray(board_angles_during_flex, dtype=float)
    change = float(wrap(a[-1] - a[0]))
    if abs(change) < min_change_deg:
        raise ValueError(f"only {change:+.1f} deg of motion; flex the thigh at least {min_change_deg} deg")
    return 1 if change > 0 else -1


def check_board_frame(frame: LegFrame, max_zero_deg: float = 10.0) -> None:
    """After calibrating: refuse to run if the board's own frame looks wrong.
    A wrong sign would flip the torque direction, so we stop instead of silently fixing it."""
    if frame.sign != +1:
        raise ValueError(f"{frame.leg}: board angle goes DOWN with flexion; fix joint_direction in params.json")
    if abs(frame.zero) > max_zero_deg:
        raise ValueError(f"{frame.leg}: standing reads {frame.zero:+.1f} deg (expected ~0); check joint_zero_deg in params.json")


# ---- Save / load one session's calibration -----------------------------------------------------
def save_frames(frames: dict, path) -> None:
    """{"left": LegFrame, "right": LegFrame} -> JSON. Keep one file per session next to the bags."""
    Path(path).write_text(json.dumps({leg: asdict(f) for leg, f in frames.items()}, indent=2))


def load_frames(path) -> dict:
    d = json.loads(Path(path).read_text())
    return {leg: LegFrame(**f) for leg, f in d.items()}
