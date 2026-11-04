"""EKF to see where you are in the gait.

State x (guess):           [phase, phase_rate, stride_length, incline]
Measurement z (actual):     [thigh_angle, thigh_velocity]
"""
import numpy as np

from kinematic_control.gait_model import GaitModel


class PhaseEKF:
    def __init__(self, model: GaitModel, dt: float, x0, P0, Q, R):
        """model: the fitted gait model
        dt: time between ticks (s)
        x0: starting guess
        P0: State covariance for x0 > P : state covariance for its guess
        Q: Prediction noise covariance
        R: Measurement noise covariance"""
        raise NotImplementedError

    def predict(self) -> None:
        """Move the guess forward one tick, and grow the uncertainty by Q."""
        raise NotImplementedError

    def measurement(self, x) -> np.ndarray:
        """What the sensor SHOULD read if the state were x: [expected angle, expected velocity]."""
        raise NotImplementedError

    def measurement_jacobian(self, x) -> np.ndarray:
        """How each expected reading changes when each state change"""
        raise NotImplementedErrorcoo

    def update(self, z) -> None:
        """Compare the real reading z with measurement(x) and correct the guess+covariance"""
        raise NotImplementedError

    def step(self, z) -> np.ndarray:
        """One full tick: predict, then update. Returns the new state guess."""
        raise NotImplementedError
