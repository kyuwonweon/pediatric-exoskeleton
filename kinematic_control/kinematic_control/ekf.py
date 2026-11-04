"""EKF to see where you are in the gait.

State x (guess):           [phase, phase_rate, stride_length, incline]
Measurement z (actual):     [thigh_angle, thigh_velocity]
"""
import numpy as np

from kinematic_control.gait_model import GaitModel


class PhaseEKF:
    # Realistic range of each state; anything outside is clipped back in after a correction.
    #                   phase  rate (strides/s)  stride (m)  incline (deg)
    LOWER = np.array([0.0,   0.3,              0.6,        -15.0])
    UPPER = np.array([1.0,   1.6,              2.0,        15.0])

    def __init__(self, model: GaitModel, dt: float, x0, P0, Q, R):
        """model: the fitted gait model
        dt: time between ticks (s)
        x0: starting guess
        P0: State covariance for x0 > P : state covariance for its guess
        Q: Prediction noise covariance
        R: Measurement noise covariance"""
        self.model = model
        self.dt = dt
        self.x = np.array(x0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)
        self.mahalanobis = 0.0

    def initialize(self, readings) -> None:
        """Pick the starting state from the first readings"""
        readings = np.asarray(readings, dtype=float)
        rate = self.x[1]
        phases0, strides, inclines = np.meshgrid(np.linspace(0, 1, 50, endpoint=False),
                                                 np.linspace(self.LOWER[2], self.UPPER[2], 8),
                                                 np.linspace(-12, 12, 9), indexing="ij")
        phases0, strides, inclines = phases0.ravel(), strides.ravel(), inclines.ravel()

        R_inv = np.linalg.inv(self.R)
        score = np.zeros(len(phases0))
        for t, z in enumerate(readings):    
            phase = (phases0 + rate * self.dt * t) % 1
            angle = self.model.predict(phase, strides, inclines)
            velocity = self.model.predict_dphase(phase, strides, inclines) * rate
            y = np.stack([z[0] - angle, z[1] - velocity], axis=1)
            score += np.einsum("ni,ij,nj->n", y, R_inv, y)

        best = np.argmin(score)
        t_end = len(readings) - 1
        self.x[0] = (phases0[best] + rate * self.dt * t_end) % 1
        self.x[2], self.x[3] = strides[best], inclines[best]

    def predict(self) -> None:
        """Move the guess forward one tick, and grow the uncertainty by Q."""
        # F is the prediction rule. only phase change. stride, rate and incline stay same
        # phase += phase_rate * dt
        F = np.array([[1, self.dt, 0, 0],
                      [0, 1,       0, 0],
                      [0, 0,       1, 0],
                      [0, 0,       0, 1]])
        self.x = F @ self.x
        self.x[0] %= 1.0                           # wrap phase
        self.P = F @ self.P @ F.T + self.Q         # new state covariance

    def measurement(self, x) -> np.ndarray:
        """What the sensor SHOULD read if the state were x: [expected angle, expected velocity]."""
        phase, phase_rate, stride_length, incline = x
        angle = self.model.predict(phase, stride_length, incline)[0]
        velocity = self.model.predict_dphase(phase, stride_length, incline)[0] * phase_rate
        return np.array([angle, velocity])

    def measurement_jacobian(self, x) -> np.ndarray:
        """How each expected reading changes when each state change"""
        phase, phase_rate, stride_length, incline = x

        def slope(**d):  # a slope of the gait model at this state, as one number
            return self.model.predict_slope(phase, stride_length, incline, **d)[0]

        d1 = slope(d_phase=1)  # angle' (deg per stride)
        return np.array([
            # d/dphase                      d/drate  d/dstride                              d/dincline
            [d1,                            0.0,     slope(d_stride=1),                     slope(d_incline=1)],
            [slope(d_phase=2) * phase_rate, d1,      slope(d_phase=1, d_stride=1) * phase_rate,
                                                     slope(d_phase=1, d_incline=1) * phase_rate],
        ])

    def update(self, z) -> None:
        """Compare the real reading z with measurement(x) and correct the guess+covariance"""
        H = self.measurement_jacobian(self.x)
        y = np.asarray(z, dtype=float) - self.measurement(self.x)  # innovation: real - expected
        S = H @ self.P @ H.T + self.R                               # expected spread of y
        K = self.P @ H.T @ np.linalg.inv(S)                         # Kalman gain (4 x 2)

        self.x = self.x + K @ y                                     # nudge the guess
        self.x[0] %= 1.0                                            # wrap phase
        self.x = np.clip(self.x, self.LOWER, self.UPPER)            # keep every state realistic
        self.P = (np.eye(4) - K @ H) @ self.P                       # more sure now
        self.mahalanobis = float(y @ np.linalg.inv(S) @ y)

    def step(self, z) -> np.ndarray:
        """One full tick: predict, then update. Returns the new state guess."""
        self.predict()
        self.update(z)
        return self.x.copy()
