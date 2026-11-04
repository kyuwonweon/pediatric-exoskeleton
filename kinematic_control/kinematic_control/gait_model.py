"""Create a gait model with curve fitting(least squares)

Input : phase, stride length, incline
Output: Thigh angle
"""

import json
import sys
from pathlib import Path

import numpy as np

from kinematic_control.data_loader import PHASE, Strides

MODEL_PATH = Path(__file__).parent / "models" / "gait_model.json"


class GaitModel:
    def __init__(self, n_harmonics: int = 4, poly_degree: int = 2):  # picked by leave_one_subject_out
        """n_harmonics: how many sine/cosine waves describe the shape over one stride.
        poly_degree: power of that harmonics"""
        self.n_harmonics = n_harmonics
        self.poly_degree = poly_degree
        self.weights = None

    def basis(self, phase, stride_length, incline,
              d_phase: int = 0, d_stride: int = 0, d_incline: int = 0) -> np.ndarray:
        """Build the matrix of shapes: one row per sample, one column per shape.

        Columns = every wave times every task term:
            waves      : 1, cos(2*pi*k*phase), sin(2*pi*k*phase) for k = 1..n_harmonics
            task terms : 1, stride, incline (poly_degree 1), plus stride^2, stride*incline, incline^2 (degree 2)
        d_phase / d_stride / d_incline = n gives the n-th slope of every shape with respect to that input
        instead (same weights apply). d_phase can be 0, 1 or 2; d_stride and d_incline 0 or 1.
        """
        # Accept single numbers or arrays, and make them the same length.
        phase, stride_length, incline = np.broadcast_arrays(
            np.atleast_1d(phase), np.atleast_1d(stride_length), np.atleast_1d(incline))

        # Step 1: waves from phase -> (n_samples, 2 * n_harmonics + 1)
        # Slopes: the constant 1 is flat (0). Each slope of cos/sin multiplies by 2*pi*k and shifts the
        # wave a quarter turn: cos -> -sin -> -cos, sin -> cos -> -sin.
        waves = [np.ones_like(phase) if d_phase == 0 else np.zeros_like(phase)]
        for k in range(1, self.n_harmonics + 1):
            w = 2 * np.pi * k
            shift = d_phase * np.pi / 2
            waves.append(w ** d_phase * np.cos(w * phase + shift))
            waves.append(w ** d_phase * np.sin(w * phase + shift))
        waves = np.stack(waves, axis=1)

        # Step 2: task terms from stride length and incline -> (n_samples, n_task)
        # Each term is stride^a * incline^b; its slope with respect to stride is a * stride^(a-1) * incline^b.
        task = [np.ones_like(phase) if d_stride == 0 and d_incline == 0 else np.zeros_like(phase)]
        for degree in range(1, self.poly_degree + 1):
            for i in range(degree + 1):
                a, b, coef = degree - i, i, 1.0  # stride^a * incline^b
                if d_stride:
                    coef, a = coef * a, a - 1
                if d_incline:
                    coef, b = coef * b, b - 1
                task.append(coef * stride_length ** max(a, 0) * incline ** max(b, 0))
        task = np.stack(task, axis=1)

        # Step 3: every wave times every task term, laid out in one row per sample.
        return (waves[:, :, None] * task[:, None, :]).reshape(len(phase), -1)

    def fit(self, strides: Strides) -> "GaitModel":
        """Least squares: find the weights that best reproduce every stride in the data."""
        n_strides, n_points = strides.thigh.shape
        phase = np.tile(PHASE, n_strides)
        stride_length = np.repeat(strides.stride_length, n_points)
        incline = np.repeat(strides.incline, n_points)
        y = strides.thigh.reshape(-1)

        A = self.basis(phase, stride_length, incline)
        self.weights, *_ = np.linalg.lstsq(A.T @ A, A.T @ y, rcond=None)
        return self

    def predict(self, phase, stride_length, incline) -> np.ndarray:
        """Expected thigh angle (deg)."""
        return self.basis(phase, stride_length, incline) @ self.weights

    def predict_dphase(self, phase, stride_length, incline) -> np.ndarray:
        """How fast the angle changes with phase (deg per stride)."""
        return self.predict_slope(phase, stride_length, incline, d_phase=1)

    def predict_slope(self, phase, stride_length, incline, d_phase=0, d_stride=0, d_incline=0) -> np.ndarray:
        """Any slope of the angle, e.g. d_stride=1: deg per meter of stride. The EKF Jacobian uses these."""
        return self.basis(phase, stride_length, incline, d_phase, d_stride, d_incline) @ self.weights

    def save(self, path) -> None:
        """Write settings and weights as JSON: plain text that MicroPython on the board can read too."""
        data = {"n_harmonics": self.n_harmonics, "poly_degree": self.poly_degree,
                "weights": self.weights.tolist()}
        Path(path).write_text(json.dumps(data, indent=1))

    @classmethod
    def load(cls, path) -> "GaitModel":
        data = json.loads(Path(path).read_text())
        model = cls(data["n_harmonics"], data["poly_degree"])
        model.weights = np.array(data["weights"])
        return model


def evaluate(model: GaitModel, strides: Strides) -> float:
    """RMSE (deg) between the model and the strides. Run it on subjects NOT used for fitting."""
    n_strides, n_points = strides.thigh.shape
    pred = model.predict(np.tile(PHASE, n_strides),
                         np.repeat(strides.stride_length, n_points),
                         np.repeat(strides.incline, n_points))
    return float(np.sqrt(np.mean((pred - strides.thigh.reshape(-1)) ** 2)))


def leave_one_subject_out(strides: Strides, **model_settings) -> float:
    """Fit on 9 subjects, test on the 10th, repeat for each subject; return the average test RMSE.
    This is how well the model should do on a new person, like you."""
    errors = []
    for subject in np.unique(strides.subject):
        test = strides.subject == subject
        model = GaitModel(**model_settings).fit(strides.select(~test))
        errors.append(evaluate(model, strides.select(test)))
    return float(np.mean(errors))


if __name__ == "__main__":
    from kinematic_control.data_loader import load_walking

    strides = load_walking()
    if "--sweep" in sys.argv:  # compare settings on unseen subjects
        print("test error on unseen subjects (deg):")
        for poly_degree in (1, 2):
            for n_harmonics in (2, 3, 4, 5, 6, 8):
                err = leave_one_subject_out(strides, n_harmonics=n_harmonics, poly_degree=poly_degree)
                print(f"  degree {poly_degree}, {n_harmonics} harmonics: {err:.2f}")

    # Final model: default settings, fit on all 10 subjects.
    model = GaitModel().fit(strides)
    MODEL_PATH.parent.mkdir(exist_ok=True)
    model.save(MODEL_PATH)
    print(f"saved {len(model.weights)} weights to {MODEL_PATH}")
