"""Step 6a: torque map. Where you are in the stride -> how much hip torque to command.

Shape: the normal human hip torque from Reznick et al. 2021, fit exactly like the gait model
       (waves in phase x polynomial in stride length and incline), just on torque instead of angle.
Size:  chosen by you in Nm (peak_Nm). A real hip makes ~1 Nm/kg (~70 Nm); the motor gives a few Nm,
       so we borrow the dataset's timing and shape, not its size.

Sign: + = flexion (pulls the thigh forward), - = extension (pushes it back), like the dataset.
Converting to each motor's own direction (id1 +, id2 -) happens later, at the motor output.
"""
import sys
from pathlib import Path

import numpy as np

from kinematic_control.gait_model import GaitModel

TORQUE_MODEL_PATH = Path(__file__).parent / "models" / "torque_model.json"
N_HARMONICS = 6  # sweep: 4-12 harmonics all ~0.22 Nm/kg on unseen people (person-to-person spread dominates); 6 is enough

# The range the dataset covers (middle 98% of strides). Outside it the fit is made up, so clip inputs to it.
STRIDE_RANGE = (0.85, 1.45)   # m
INCLINE_RANGE = (-10.0, 10.0)  # deg


def biggest_torque(model: GaitModel) -> float:
    """Largest |torque| (Nm/kg) the model gives anywhere inside the dataset range.
    Dividing by it makes the whole map run from -1 to +1."""
    phase, stride, incline = np.meshgrid(np.linspace(0, 1, 200, endpoint=False),
                                         np.linspace(*STRIDE_RANGE, 7),
                                         np.linspace(*INCLINE_RANGE, 9), indexing="ij")
    return float(np.max(np.abs(model.predict(phase.ravel(), stride.ravel(), incline.ravel()))))


class TorqueMap:
    def __init__(self, model: GaitModel, peak_Nm: float = 2.0, lead_s: float = 0.0):
        """model: torque model (GaitModel fit with signal="torque")
        peak_Nm: the largest torque the map will ever ask for (steepest uphill, longest stride).
                 Level walking comes out smaller, so the assist grows uphill like a real hip's does.
        lead_s: look this many seconds ahead in the stride, to make up for the motor's delay."""
        self.model = model
        self.peak_Nm = peak_Nm
        self.lead_s = lead_s
        self.scale = 1.0 / biggest_torque(model)

    def normal_torque(self, phase, stride_length, incline):
        """What a normal hip produces at this point (Nm/kg), inputs clipped to the dataset range."""
        stride_length = np.clip(stride_length, *STRIDE_RANGE)
        incline = np.clip(incline, *INCLINE_RANGE)
        return self.model.predict(phase, stride_length, incline)

    def command(self, phase, phase_rate, stride_length, incline) -> float:
        """Torque to command now (Nm, + = flexion), from the EKF's state estimate."""
        phase_ahead = (phase + phase_rate * self.lead_s) % 1.0
        return float(self.peak_Nm * self.scale * self.normal_torque(phase_ahead, stride_length, incline)[0])

    @classmethod
    def load(cls, path=TORQUE_MODEL_PATH, **settings) -> "TorqueMap":
        return cls(GaitModel.load(path), **settings)


def plot_fit(model: GaitModel, strides, path) -> None:
    """Dataset vs model at 1.0 m/s for each incline: does the map have the right shape and timing?"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from kinematic_control.data_loader import PHASE

    inclines = np.unique(strides.incline)
    fig, axes = plt.subplots(1, len(inclines), figsize=(3.2 * len(inclines), 3.4), sharey=True)
    for ax, incline in zip(axes, inclines):
        k = (strides.speed == 1.0) & (strides.incline == incline)
        per_person = np.array([strides.torque[k & (strides.subject == u)].mean(0)
                               for u in np.unique(strides.subject[k])])
        x = 100 * PHASE
        ax.fill_between(x, per_person.min(0), per_person.max(0), color="0.85", lw=0, label="range of people")
        ax.plot(x, per_person.mean(0), color="0.15", lw=2, label="dataset average")
        ax.plot(x, model.predict(PHASE, strides.stride_length[k].mean(), incline), color="#2a6fdb", lw=2,
                ls="--", label="torque map")
        ax.axhline(0, color="0.6", lw=0.8)
        ax.set_title(f"{incline:+.0f} deg", fontsize=10)
        ax.set_xlabel("stride (%)  0 = heel strike")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("hip torque (Nm/kg)\n+ flexion  /  - extension")
    axes[-1].legend(fontsize=8, loc="lower right")
    fig.suptitle("Hip torque at 1.0 m/s: dataset vs torque map", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    from kinematic_control.data_loader import load_walking
    from kinematic_control.gait_model import leave_one_subject_out

    strides = load_walking()
    if "--sweep" in sys.argv:  # compare settings on unseen subjects
        print("torque test error on unseen subjects (Nm/kg):")
        for poly_degree in (1, 2):
            for n_harmonics in (4, 6, 8, 10, 12):
                err = leave_one_subject_out(strides, "torque", n_harmonics=n_harmonics, poly_degree=poly_degree)
                print(f"  degree {poly_degree}, {n_harmonics:2d} harmonics: {err:.3f}")

    model = GaitModel(n_harmonics=N_HARMONICS, poly_degree=2).fit(strides, signal="torque")
    model.save(TORQUE_MODEL_PATH)
    print(f"saved {len(model.weights)} weights to {TORQUE_MODEL_PATH}")

    plot_path = Path(__file__).parent.parent / "plots" / "torque_map.png"
    plot_fit(model, strides, plot_path)
    print(f"saved {plot_path}")

    # What the motor would be asked for, with peak_Nm = 2.
    tmap = TorqueMap(model, peak_Nm=2.0)
    phase = np.linspace(0, 1, 200, endpoint=False)
    for incline in (-10, 0, 10):
        tau = [tmap.command(p, 0.9, 1.15, incline) for p in phase]
        print(f"  {incline:+3d} deg, stride 1.15 m: {min(tau):+.2f} to {max(tau):+.2f} Nm")
