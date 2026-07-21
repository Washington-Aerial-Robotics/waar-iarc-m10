from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


class ImuAttitudeFilter:
    """Complementary filter for roll/pitch from accel, yaw from gyro integration."""

    def __init__(self, alpha: float = 0.98):
        self.alpha = alpha
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.yaw_rad = 0.0
        self._last_timestamp: float | None = None

    def update(self, accel_m_s2: np.ndarray, gyro_rad_s: np.ndarray, timestamp: float) -> np.ndarray:
        ax, ay, az = accel_m_s2
        gx, gy, gz = gyro_rad_s

        if abs(az) > 0.1 or abs(ax) > 0.1 or abs(ay) > 0.1:
            roll_meas = float(np.arctan2(ay, az))
            pitch_meas = float(np.arctan2(-ax, np.sqrt(ay * ay + az * az)))
        else:
            roll_meas = self.roll_rad
            pitch_meas = self.pitch_rad

        if self._last_timestamp is not None:
            dt = max(1e-4, timestamp - self._last_timestamp)
            self.roll_rad = self.alpha * (self.roll_rad + gx * dt) + (1.0 - self.alpha) * roll_meas
            self.pitch_rad = self.alpha * (self.pitch_rad + gy * dt) + (1.0 - self.alpha) * pitch_meas
            self.yaw_rad += gz * dt
        else:
            self.roll_rad = roll_meas
            self.pitch_rad = pitch_meas
            self.yaw_rad = 0.0

        self._last_timestamp = timestamp
        return np.array([self.yaw_rad, self.pitch_rad, self.roll_rad], dtype=np.float64)

    def quaternion_xyzw(self) -> np.ndarray:
        rot = R.from_euler("zyx", [self.yaw_rad, self.pitch_rad, self.roll_rad])
        return rot.as_quat()


class ImuVelocityIntegrator:
    """Light velocity estimate from body-frame acceleration (high drift — VO dominates)."""

    def __init__(self):
        self.velocity = np.zeros(3, dtype=np.float64)
        self._last_timestamp: float | None = None

    def update(
        self,
        accel_m_s2: np.ndarray,
        orientation_euler: np.ndarray,
        timestamp: float,
        *,
        weight: float = 0.05,
    ) -> np.ndarray:
        if self._last_timestamp is None:
            self._last_timestamp = timestamp
            return self.velocity.copy()

        dt = max(1e-4, timestamp - self._last_timestamp)
        self._last_timestamp = timestamp

        yaw, pitch, roll = orientation_euler
        rot = R.from_euler("zyx", [yaw, pitch, roll])
        world_accel = rot.apply(accel_m_s2)
        world_accel[2] -= 9.81  # remove gravity when roughly upright

        self.velocity += world_accel * dt * weight
        return self.velocity.copy()
