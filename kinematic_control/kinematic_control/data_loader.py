"""Step 1: load the Reznick et al. 2021 walking data and turn it into flat arrays of strides.

Dataset layout (from its README):
    Normalized.<subject>.Walk.<speed>.<incline>.<dataType>.<variable>
    subject : AB01 ... AB10
    speed   : 's0x8', 's1', 's1x2'   ('x' is the decimal point -> 0.8, 1.0, 1.2 m/s)
    incline : 'i0', 'i5', 'i10' uphill, 'in5', 'in10' downhill
    each signal: 150 points per stride (heel strike to heel strike) x 3 axes x N strides
    The file is MATLAB v7.3 (HDF5), so h5py reads it, with the axes reversed: (N strides, 3 axes, 150 points).
    Axis 0 is sagittal. Strides of both legs are stacked together.
    events.StrideDetails rows: start frame, end frame, stride time in frames (100 Hz), leg (1 or 2).

What we want out: one row per stride, so the gait model can be fit with plain least squares.
    thigh[k, :]   global thigh angle over the stride (deg), 150 points; + is flexion (thigh forward)
    torque[k, :]  internal hip moment over the stride (Nm/kg); + is flexion, - is extension
    speed[k], incline[k], stride_time[k], stride_length[k], leg[k], subject[k]
"""
from dataclasses import dataclass
from pathlib import Path

import h5py
import numpy as np

DEFAULT_PATH = Path.home() / "Desktop/projects/final/data/reznick/Normalized.mat"
N_POINTS = 150                       # samples per normalized stride
PHASE = np.linspace(0, 1, N_POINTS)  # 0 = heel strike, 1 = next heel strike


STEADY_SPEEDS = ("s0x8", "s1", "s1x2")  # skip the a*/d* acceleration trials
SAGITTAL = 0


@dataclass
class Strides:
    thigh: np.ndarray          # (N, 150) deg
    torque: np.ndarray         # (N, 150) Nm/kg
    speed: np.ndarray          # (N,) m/s
    incline: np.ndarray        # (N,) deg, + is uphill
    stride_time: np.ndarray    # (N,) s
    stride_length: np.ndarray  # (N,) m, = speed * stride time
    leg: np.ndarray            # (N,) 1 or 2, as labeled in the dataset
    subject: np.ndarray        # (N,) e.g. 'AB01'

    def select(self, mask) -> "Strides":
        """Keep only the strides where mask is True, in every field at once."""
        return Strides(**{name: getattr(self, name)[mask] for name in self.__dataclass_fields__})


def parse_speed(key: str) -> float:
    """'s1x2' -> 1.2"""
    return float(key[1:].replace("x", "."))


def parse_incline(key: str) -> float:
    """'i10' -> +10, 'in10' -> -10, 'i0' -> 0"""
    return -float(key[2:]) if key.startswith("in") else float(key[1:])


def load_walking(path: Path = DEFAULT_PATH) -> Strides:
    """Read every steady walking stride of every subject into one Strides object."""
    cols = {name: [] for name in Strides.__dataclass_fields__}
    with h5py.File(path, "r") as f:
        root = f["Normalized"]
        for subject in root:
            walk = root[subject]["Walk"]
            for speed_key in STEADY_SPEEDS:
                if speed_key not in walk:
                    continue
                for incline_key in walk[speed_key]:
                    trial = walk[speed_key][incline_key]
                    hip = trial["jointAngles/HipAngles"][:, SAGITTAL, :]       # hip relative to pelvis
                    pelvis = trial["jointAngles/PelvisAngles"][:, SAGITTAL, :]  # pelvis relative to the room
                    moment = trial["jointMoments/HipMoment"][:, SAGITTAL, :]
                    details = trial["events/StrideDetails"][()]
                    n = hip.shape[0]

                    cols["thigh"].append(hip - pelvis)  # same as the README's example script
                    cols["torque"].append(moment)
                    cols["speed"].append(np.full(n, parse_speed(speed_key)))
                    cols["incline"].append(np.full(n, parse_incline(incline_key)))
                    cols["stride_time"].append(details[2] / 100.0)
                    cols["leg"].append(details[3].astype(int))
                    cols["subject"].append(np.full(n, subject))

    out = {name: np.concatenate(parts) for name, parts in cols.items() if parts}
    out["stride_length"] = out["speed"] * out["stride_time"]
    strides = Strides(**out)

    # Motion capture drops markers now and then; throw away strides with any gap.
    ok = np.isfinite(strides.thigh).all(1) & np.isfinite(strides.torque).all(1)
    return strides.select(ok)


if __name__ == "__main__":
    s = load_walking()
    print(f"{len(s.speed)} strides from {len(set(s.subject))} subjects")
    print(f"stride time {s.stride_time.min():.2f}-{s.stride_time.max():.2f} s, "
          f"stride length {s.stride_length.min():.2f}-{s.stride_length.max():.2f} m")
    for v in np.unique(s.speed):
        for i in np.unique(s.incline):
            m = (s.speed == v) & (s.incline == i)
            print(f"  {v:.1f} m/s {i:+3.0f} deg: {m.sum():4d} strides, thigh range "
                  f"{s.thigh[m].min(1).mean():+.1f} to {s.thigh[m].max(1).mean():+.1f} deg, "
                  f"peak flexion torque {s.torque[m].max(1).mean():.2f} Nm/kg")
