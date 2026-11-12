"""Board joint angle -> anatomical hip angle, per leg.

The boards report a "joint angle" (robotCMBenchTop._servo_to_joint_angle):
    board = wrap(joint_direction * (servo - joint_zero_deg))
but that frame is not anatomical (full extension reads ~19.9 deg, not 0) and the two legs are
~35 deg apart on the bench. We don't change the boards; we convert on our side:

    hip = wrap(sign * (board - zero))

    zero: the board angle while you stand still and upright (measured each session, per leg)
    sign: +1 if the board angle goes UP when the thigh flexes (forward), -1 if it goes down

Result, same meaning on both legs: 0 = standing upright, + = flexion (thigh forward), deg.
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
    zero: float         # board angle while standing upright (deg)
    sign: int = +1      # +1: board angle increases with flexion; -1: it decreases

    def hip_angle(self, board_angle):
        """Board joint angle (deg) -> anatomical hip angle (deg, 0 = standing, + = flexion)."""
        return wrap(self.sign * (np.asarray(board_angle) - self.zero))

    def hip_velocity(self, board_velocity):
        """Board joint velocity (deg/s) -> anatomical (deg/s). Only the sign changes."""
        return self.sign * np.asarray(board_velocity)

    def joint_torque(self, hip_torque):
        """Anatomical torque (Nm, + = flexion) -> board joint torque (the board's own sign).
        Same sign rule as the angle: if the board's angle runs backward, so does its torque."""
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


# ---- Save / load one session's calibration -----------------------------------------------------
def save_frames(frames: dict, path) -> None:
    """{"left": LegFrame, "right": LegFrame} -> JSON. Keep one file per session next to the bags."""
    Path(path).write_text(json.dumps({leg: asdict(f) for leg, f in frames.items()}, indent=2))


def load_frames(path) -> dict:
    d = json.loads(Path(path).read_text())
    return {leg: LegFrame(**f) for leg, f in d.items()}
