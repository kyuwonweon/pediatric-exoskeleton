"""Gait Phase Estimation

Method: Quintero, Lambert, Villarreal & Gregg
"Real-Time Continuous Gait Phase and Speed Estimation from a Single Sensor"
https://locolab.robotics.umich.edu/documents/CCTA17_0285_FI.pdf
"""
import json
from collections import deque
from pathlib import Path

import numpy as np

from kinematic_control.data_loader import Strides

TWO_PI = 2 * np.pi
LOOKUP_PATH = Path(__file__).resolve().parents[2] / "models" / "phase_lookup.json"


def wrap(phi):
    """Any angle -> [0, 2pi)."""
    return phi % TWO_PI


def signed_diff(a, b):
    """a - b on a circle, in (-pi, pi]. So 0.1 - 6.2 = +0.18 (a small step forward across 0)."""
    return (a - b + np.pi) % TWO_PI - np.pi


class PeakDetector:
    """Finds the thigh angle's peaks (max flexion) and valleys (max extension), one each per stride."""

    def __init__(self, start_threshold: float = 5.0, fraction: float = 0.25):
        self.start_threshold = start_threshold
        self.fraction = fraction
        self.hi = None         # last peak value
        self.lo = None         # last valley value
        self.candidate = None  # best peak (if rising) or valley (if falling) since the last turn
        self.rising = True
        self.ticks_since_turn = 0

    @property
    def reversal_threshold(self) -> float:
        """Candidate peak only counts as the real peak once the angle has dropped more than that threshold 
        and a candidate valley counts once the angle has risen more"""
        if self.hi is None or self.lo is None:
            return self.start_threshold
        return self.fraction * (self.hi - self.lo)

    def update(self, v: float):
        """Returns "peak" or "valley" on the tick one is confirmed, else None."""
        self.ticks_since_turn += 1
        if self.candidate is None:
            self.candidate = v
            return None
        if self.rising:
            if v > self.candidate:
                self.candidate = v
            elif v < self.candidate - self.reversal_threshold:
                self.hi, self.candidate, self.rising = self.candidate, v, False
                self.ticks_since_turn = 0
                return "peak"
        else:
            if v < self.candidate:
                self.candidate = v
            elif v > self.candidate + self.reversal_threshold:
                self.lo, self.candidate, self.rising = self.candidate, v, True
                self.ticks_since_turn = 0
                return "valley"
        return None


class RunningStats:
    """Mean and std of a signal over strides"""

    def __init__(self):
        self.mean = None
        self.std = None
        self._reset()

    def _reset(self):
        self.n, self.s, self.s2 = 0, 0.0, 0.0

    def add(self, v: float) -> None:
        self.n += 1
        self.s += v
        self.s2 += v * v

    def close(self, min_samples: int = 10) -> None:
        if self.n >= min_samples:
            self.mean = self.s / self.n
            self.std = max(np.sqrt(max(self.s2 / self.n - self.mean ** 2, 0.0)), 1e-9)
        self._reset()

    @property
    def ready(self) -> bool:
        return self.mean is not None


class PhasePortrait:
    def __init__(self, dt: float = 0.02, window: int = 6, degree: int = 2,
                 y_signal: str = "integral", leg_sign: int = +1,
                 max_still_s: float = 1.5, still_s: float = 0.4, still_frac: float = 0.12,
                 resume_tol: float = 0.3, lookup=None):
        """dt: time between ticks (s); 0.02 = 50 Hz like the exo
        window: how many past angles the smoother fits (6 = 0.12 s). Bigger = smoother, more delay
        degree: polynomial degree for the smoother
        y_signal: "integral" (default, better) or "velocity" (the CCTA 2017 paper)
        leg_sign: +1 if the angle is already + = flexion (dataset frame). -1 flips x, for a sensor
                  mounted mirrored (the paper's left leg). Better: convert upstream and keep +1.
        max_still_s: no thigh peak/valley for this long = not walking (slow backup check)
        still_s, still_frac: thigh angle stayed within still_frac x last stride's swing for the
                  last still_s = not walking (fast check). Swept on Reznick walks: 0.4 s / 12 %
                  -> ~0.06 % false stops while walking, stop caught ~0.26 s after the thigh rests.
        resume_tol: after a stop, phase stays held until the live phase is this close to it
        lookup: table that correlates and tells the gait phase based on the phi angle from the circle
        """
        if y_signal not in ("integral", "velocity"):
            raise ValueError("y_signal must be 'integral' or 'velocity'")
        self.dt = dt
        self.y_signal = y_signal
        self.leg_sign = leg_sign
        self.max_still_ticks = int(max_still_s / dt)
        self.recent = deque(maxlen=int(still_s / dt))   # last still_s of smoothed angles
        self.still_frac = still_frac
        self.resume_tol = resume_tol
        self.lookup = lookup

        self.angles = deque(maxlen=window)
        t = (np.arange(window) - (window - 1)) * dt
        vander = np.vander(t, degree + 1, increasing=True)
        self.weights = np.linalg.pinv(vander)[:2]

        self.peaks = PeakDetector()
        self.angle_stats = RunningStats()     # x: thigh angle
        self.y_stats = RunningStats()         # y: integral or velocity
        self.integral = 0.0                   # running integral of (angle - last stride's mean)

        self.last_phi = None
        self.held = False

        self.stopped = False
        self.phi = None
        self.x = self.y = 0.0

    @property
    def ready(self) -> bool:
        """True once the stats of a full stride are known for both x and y"""
        return self.angle_stats.ready and self.y_stats.ready

    def smooth(self, angle: float) -> tuple[float, float]:
        """Add one raw angle; fit a polynomial to the collected angles.
        Returns (theta, theta_dot)"""
        self.angles.append(angle)
        if len(self.angles) < self.angles.maxlen:
            return angle, 0.0
        theta, theta_dot = self.weights @ np.asarray(self.angles)
        return float(theta), float(theta_dot)

    def update_stride_stats(self, theta: float, theta_dot: float) -> float:
        """Returns y value (before centering) the integral or the velocity."""
        if self.y_signal == "integral":
            if self.angle_stats.ready:
                self.integral += (theta - self.angle_stats.mean) * self.dt
            y = self.integral
        else:
            y = theta_dot
        self.angle_stats.add(theta)
        if self.angle_stats.ready or self.y_signal == "velocity":
            self.y_stats.add(y)      # integral only means something once the angle mean is known
        if self.peaks.update(theta) == "peak":
            self.angle_stats.close()
            self.y_stats.close()
        return y

    def normalize(self, theta: float, y: float) -> tuple[float, float]:
        """Center each signal on last stride's mean and divide by its std"""
        a, s = self.angle_stats, self.y_stats
        x = self.leg_sign * (theta - a.mean) / a.std
        y = (y - s.mean) / s.std
        return x, (y if self.y_signal == "integral" else -y)

    @staticmethod
    def raw_phase(x: float, y: float) -> float:
        """phi = atan2(y, x), wrapped into [0, 2*pi). Starts at max thigh flexion, not heel strike."""
        return float(wrap(np.arctan2(y, x)))

    def is_stopped(self, theta: float) -> bool:
        """Stopped if the thigh angle has stayed inside a small band for the last still_s (fast:
        band = still_frac x last stride's swing, ~5 deg), or hasn't turned around for max_still_s
        (backup). Uses the angle, not velocity: velocity amplifies sensor noise (0.5 deg of angle
        noise gave 15-33 deg/s of velocity jitter while standing, so a velocity test never fired)."""
        self.recent.append(theta)
        full = len(self.recent) == self.recent.maxlen
        still = full and (max(self.recent) - min(self.recent)) < self.still_frac * self.stride_size
        stale = self.peaks.ticks_since_turn > self.max_still_ticks
        return still or stale

    def hold(self, phi: float, stopped: bool) -> float:
        """Return the phase to output."""
        if self.last_phi is None:
            self.last_phi = phi
            return phi
        step = signed_diff(phi, self.last_phi)
        if stopped:
            self.held = True
        elif self.held and abs(step) <= self.resume_tol:
            self.held = False
        if not self.held and step > 0:
            self.last_phi = phi
        return self.last_phi

    def to_gait_phase(self, phi: float) -> float:
        """Convert phi into phase 0-1"""
        if self.lookup is None:
            return phi / TWO_PI
        grid, phase = self.lookup
        g = np.concatenate([grid - TWO_PI, grid, grid + TWO_PI])
        p = np.concatenate([phase - 1, phase, phase + 1])
        return float(np.interp(phi, g, p) % 1.0)

    def step(self, angle: float):
        """One tick: raw thigh angle (deg) in, gait phase (0..1) out, or None while warming up."""
        theta, theta_dot = self.smooth(angle)
        y = self.update_stride_stats(theta, theta_dot)
        if not self.ready:
            return None
        self.x, self.y = self.normalize(theta, y)
        self.stopped = self.is_stopped(theta)
        self.phi = self.hold(self.raw_phase(self.x, self.y), self.stopped)
        return self.to_gait_phase(self.phi)

    @property
    def walking(self) -> bool:
        return self.ready and not self.stopped and not self.held

    @property
    def stride_size(self) -> float:
        if self.peaks.hi is None or self.peaks.lo is None:
            return 0.0
        return self.peaks.hi - self.peaks.lo

"""For testing purposes"""
def run(walk_angle: np.ndarray, **settings) -> tuple[np.ndarray, np.ndarray]:
    """Feed a whole angle recording through a fresh PhasePortrait, one sample at a time.
    Returns (phase, phi) per tick, NaN while warming up."""
    pp = PhasePortrait(**settings)
    phase = np.full(len(walk_angle), np.nan)
    phi = np.full(len(walk_angle), np.nan)
    for k, angle in enumerate(walk_angle):
        out = pp.step(angle)
        if out is not None:
            phase[k], phi[k] = out, pp.phi
    return phase, phi


def steady_walks(strides: Strides, min_strides: int = 8):
    """Every (subject, speed, incline) as its own 50 Hz walk, leg 1. Yields (subject, Walk)."""
    from kinematic_control.replay import make_walk  # here, not at the top: avoids a circular import later

    for subject in np.unique(strides.subject):
        for speed in np.unique(strides.speed):
            for incline in np.unique(strides.incline):
                m = ((strides.subject == subject) & (strides.speed == speed)
                     & (strides.incline == incline) & (strides.leg == 1))
                if m.sum() >= min_strides:
                    yield subject, make_walk(strides.select(m))


def fit_phase_lookup(strides: Strides, n_bins: int = 100, skip_s: float = 4.0,
                     **settings) -> tuple[np.ndarray, np.ndarray]:
    """Run PhasePortrait over dataset walks where the true phase is known,
    and record method phi vs true phase. Returns (phi_grid, true_phase at each grid point),
    ready for to_gait_phase(). Fit on training subjects only; test on held-out ones.

    Per bin of phi, the true phase is averaged on the circle (so 0.99 and 0.01 average to 0,
    not 0.5), then unwrapped so it climbs smoothly by 1 over the lap."""
    phis, truths = [], []
    for _, walk in steady_walks(strides):
        _, phi = run(walk.angle, **settings)
        keep = np.isfinite(phi)
        keep[: int(skip_s / settings.get("dt", 0.02))] = False
        phis.append(phi[keep])
        truths.append(walk.phase[keep])
    phi, truth = np.concatenate(phis), np.concatenate(truths)

    edges = np.linspace(0, TWO_PI, n_bins + 1)
    which = np.clip(np.digitize(phi, edges) - 1, 0, n_bins - 1)
    grid, phase = [], []
    for b in range(n_bins):
        in_bin = truth[which == b]
        if len(in_bin) == 0:
            continue  # empty bin: interpolation fills the gap
        grid.append((edges[b] + edges[b + 1]) / 2)
        phase.append(np.angle(np.mean(np.exp(1j * TWO_PI * in_bin))) / TWO_PI)  # circular mean
    phase = np.unwrap(TWO_PI * np.asarray(phase)) / TWO_PI
    return np.asarray(grid), phase


def save_lookup(lookup, path=LOOKUP_PATH) -> None:
    """Write the lookup to JSON (models/phase_lookup.json), like the gait model."""
    grid, phase = lookup
    Path(path).write_text(json.dumps({"phi": grid.tolist(), "phase": phase.tolist()}))


def load_lookup(path=LOOKUP_PATH):
    """Read the lookup back from JSON."""
    d = json.loads(Path(path).read_text())
    return np.asarray(d["phi"]), np.asarray(d["phase"])

def evaluate_held_out(strides: Strides, skip_s: float = 4.0, verbose: bool = False, **settings) -> dict:
    """For each person: lookup fit on everyone else, then mean phase error (% of stride) on them.
    Also scores the raw phi/2pi (no lookup) to show what the lookup buys."""
    from kinematic_control.replay import phase_error

    skip = int(skip_s / settings.get("dt", 0.02))
    with_lookup, without = [], []
    for subject in np.unique(strides.subject):
        lookup = fit_phase_lookup(strides.select(strides.subject != subject), **settings)
        errs, raw_errs = [], []
        for _, walk in steady_walks(strides.select(strides.subject == subject)):
            phase, phi = run(walk.angle, lookup=lookup, **settings)
            ok = np.isfinite(phase)
            ok[:skip] = False
            errs.append(phase_error(phase[ok], walk.phase[ok]))
            raw_errs.append(phase_error(phi[ok] / TWO_PI, walk.phase[ok]))
        with_lookup.append(100 * np.mean(np.concatenate(errs)))
        without.append(100 * np.mean(np.concatenate(raw_errs)))
        if verbose:
            print(f"  {subject}: phase error {with_lookup[-1]:4.1f}%  (no lookup {without[-1]:4.1f}%)")
    return {"phase_%": round(float(np.mean(with_lookup)), 2),
            "phase_%_no_lookup": round(float(np.mean(without)), 2)}


def main():
    """Run from kinematic_control/:  python -m kinematic_control.estimation.phase_portrait"""
    from kinematic_control.data_loader import load_walking

    s = load_walking()
    flat = s.select(s.incline == 0)
    for y_signal in ("velocity", "integral"):
        print(f"y = {y_signal}")
        print("  flat ground:", evaluate_held_out(flat, y_signal=y_signal))
        print("  all inclines:", evaluate_held_out(s, y_signal=y_signal, verbose=True))
    save_lookup(fit_phase_lookup(s))
    print(f"saved {LOOKUP_PATH}")
