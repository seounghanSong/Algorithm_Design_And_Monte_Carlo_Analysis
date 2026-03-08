import numpy as np
from models.estimated_target import EstimatedTarget3D

class TargetStateEKF:
    def __init__(self, x0, P0=None, q=1.0, r=25.0):
        self.x = np.array(x0, dtype=float)

        if P0 is None:
            self.P = np.eye(6) * 100.0
        else:
            self.P = np.array(P0, dtype=float)

        # process noise scale
        self.q = q

        # measurement noise variance
        self.r = r

    def predict(self, dt):
        F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1],
        ], dtype=float)

        G = np.array([
            [0.5 * dt**2, 0, 0],
            [0, 0.5 * dt**2, 0],
            [0, 0, 0.5 * dt**2],
            [dt, 0, 0],
            [0, dt, 0],
            [0, 0, dt],
        ], dtype=float)

        Q = self.q * (G @ G.T)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    # Initial EKF(linear)
    def update(self, z):
        z = np.array(z, dtype=float)

        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=float)

        R = np.eye(3) * self.r

        # innovation
        y = z - (H @ self.x)

        # innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P

    def get_estimated_target(self):
        return EstimatedTarget3D(self.x.copy())

    def get_state(self):
        return self.x.copy()