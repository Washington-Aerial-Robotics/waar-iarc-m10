from __future__ import annotations

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


class VisualOdometry:
    """
    Monocular visual odometry via Lucas-Kanade optical flow.

    Returns body-frame translation delta (forward, right, down) in meters.
    Scale uses assumed flight altitude when metric depth is unavailable.
    """

    def __init__(
        self,
        fx: float,
        fy: float,
        max_corners: int = 200,
        altitude_m: float = 1.5,
    ):
        self.fx = fx
        self.fy = fy
        self.max_corners = max_corners
        self.altitude_m = altitude_m
        self._prev_gray: np.ndarray | None = None
        self._prev_points: np.ndarray | None = None

    def reset(self) -> None:
        self._prev_gray = None
        self._prev_points = None

    def update(
        self,
        frame_bgr: np.ndarray,
        orientation_euler: np.ndarray,
        timestamp: float,
    ) -> np.ndarray | None:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        if self._prev_gray is None:
            self._prev_gray = gray
            self._prev_points = self._detect_features(gray)
            return None

        if self._prev_points is None or len(self._prev_points) < 8:
            self._prev_gray = gray
            self._prev_points = self._detect_features(gray)
            return None

        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray,
            gray,
            self._prev_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
        )

        if next_points is None or status is None:
            self._prev_gray = gray
            self._prev_points = self._detect_features(gray)
            return None

        mask = status.reshape(-1) == 1
        if mask.sum() < 8:
            self._prev_gray = gray
            self._prev_points = self._detect_features(gray)
            return None

        prev = self._prev_points[mask].reshape(-1, 2)
        nxt = next_points[mask].reshape(-1, 2)
        flow = nxt - prev

        # Median pixel motion → metric delta using pinhole model at altitude
        du = float(np.median(flow[:, 0]))
        dv = float(np.median(flow[:, 1]))
        z = max(0.5, self.altitude_m)

        # Camera frame: x right, y down, z forward
        dx_cam = -du * z / self.fx
        dy_cam = dv * z / self.fy
        dz_cam = 0.0  # monocular — no forward scale from flow alone

        delta_cam = np.array([dx_cam, dy_cam, dz_cam], dtype=np.float64)

        yaw, pitch, roll = orientation_euler
        rot = R.from_euler("zyx", [yaw, pitch, roll])
        delta_world = rot.apply(delta_cam)

        self._prev_gray = gray
        self._prev_points = self._detect_features(gray)
        return delta_world

    def _detect_features(self, gray: np.ndarray) -> np.ndarray | None:
        points = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=0.01,
            minDistance=8,
            blockSize=7,
        )
        if points is None:
            return None
        return points.astype(np.float32)
