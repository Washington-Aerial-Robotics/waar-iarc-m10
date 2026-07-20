from __future__ import annotations

import cv2
import numpy as np


class TagTrack:
    """Constant-velocity Kalman filter for 3D position (meters)."""

    def __init__(self, initial_xyz: np.ndarray, t0: float):
        self.kf = cv2.KalmanFilter(6, 3)
        self.kf.measurementMatrix = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=np.float32,
        )
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-3
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 5e-3
        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 0.1
        self.kf.statePost = np.array(
            [
                [initial_xyz[0]],
                [initial_xyz[1]],
                [initial_xyz[2]],
                [0],
                [0],
                [0],
            ],
            dtype=np.float32,
        )
        self.last_t = t0
        self.misses = 0

    def _set_transition(self, dt: float) -> None:
        self.kf.transitionMatrix = np.array(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

    def predict(self, t: float) -> np.ndarray:
        dt = max(1e-3, float(t - self.last_t))
        self._set_transition(dt)
        pred = self.kf.predict()
        self.last_t = t
        return pred[:3, 0].copy()

    def update(self, xyz: np.ndarray, t: float) -> np.ndarray:
        _ = self.predict(t)
        meas = np.array([[xyz[0]], [xyz[1]], [xyz[2]]], dtype=np.float32)
        est = self.kf.correct(meas)
        self.misses = 0
        return est[:3, 0].copy()

    def mark_missed(self) -> None:
        self.misses += 1
