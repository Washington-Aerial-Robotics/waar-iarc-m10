from __future__ import annotations

import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from .imu import ImuAttitudeFilter, ImuVelocityIntegrator
from .visual_odometry import VisualOdometry


class PoseFusion:
    """
    Fuse launch pose + IMU attitude + visual odometry deltas.

    Optional AprilTag loop-closure corrections when re-detecting known mines.
    """

    def __init__(
        self,
        launch_position: np.ndarray,
        launch_quaternion: np.ndarray,
        fx: float,
        fy: float,
        vo_altitude_m: float = 1.5,
        vo_weight: float = 0.85,
        tag_correction_gain: float = 0.25,
    ):
        self.position = launch_position.astype(np.float64).reshape(3).copy()
        self.velocity = np.zeros(3, dtype=np.float64)
        launch_rot = R.from_quat(launch_quaternion.reshape(4))
        yaw, pitch, roll = launch_rot.as_euler("zyx")
        self.orientation_euler = np.array([yaw, pitch, roll], dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)

        self._attitude_filter = ImuAttitudeFilter()
        self._attitude_filter.roll_rad = roll
        self._attitude_filter.pitch_rad = pitch
        self._attitude_filter.yaw_rad = yaw

        self._velocity_integrator = ImuVelocityIntegrator()
        self._vo = VisualOdometry(fx=fx, fy=fy, altitude_m=vo_altitude_m)
        self.vo_weight = vo_weight
        self.tag_correction_gain = tag_correction_gain
        self._last_timestamp = time.time()
        self.correction_count = 0

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        return R.from_euler("zyx", self.orientation_euler).as_quat()

    def update_imu(
        self,
        accel_m_s2: np.ndarray,
        gyro_rad_s: np.ndarray,
        timestamp: float,
    ) -> None:
        self.orientation_euler = self._attitude_filter.update(accel_m_s2, gyro_rad_s, timestamp)
        self.angular_velocity = gyro_rad_s.astype(np.float64).reshape(3)
        imu_vel = self._velocity_integrator.update(accel_m_s2, self.orientation_euler, timestamp)
        self.velocity = 0.9 * self.velocity + 0.1 * imu_vel

    def update_visual(self, frame_bgr: np.ndarray, timestamp: float) -> None:
        delta = self._vo.update(frame_bgr, self.orientation_euler, timestamp)
        if delta is None:
            return
        self.position += delta * self.vo_weight
        dt = max(1e-4, timestamp - self._last_timestamp)
        self.velocity = 0.7 * self.velocity + 0.3 * (delta / dt)
        self._last_timestamp = timestamp

    def apply_tag_correction(
        self,
        measured_world: np.ndarray,
        fused_world: np.ndarray,
        confidence: float,
    ) -> None:
        """
        Nudge pose when a re-seen tag disagrees with the fused mine position.

        Assumes error is mostly translational (reasonable at low tilt).
        """
        if confidence < 0.2:
            return
        error = fused_world.reshape(3) - measured_world.reshape(3)
        correction = error * self.tag_correction_gain * min(1.0, confidence)
        self.position += correction
        self.correction_count += 1

    def snapshot(self) -> dict:
        return {
            "position": self.position.copy(),
            "velocity": self.velocity.copy(),
            "orientation_euler": self.orientation_euler.copy(),
            "angular_velocity": self.angular_velocity.copy(),
            "quaternion_xyzw": self.quaternion_xyzw.copy(),
            "correction_count": self.correction_count,
        }
