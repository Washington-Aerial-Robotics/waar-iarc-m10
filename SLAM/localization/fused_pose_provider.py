from __future__ import annotations

import time

import numpy as np

from apriltag.models import PoseEstimate

from .esp32_comms import Esp32Client
from .fusion import PoseFusion


class FusedPoseProvider:
    """
    Onboard fusion pose provider: IMU + visual odometry on Pi, push to ESP32.

    Call update_frame() each camera frame from the perception pipeline.
  """

    def __init__(
        self,
        host: str,
        port: int = 23,
        drone_id: int = 65,
        launch_position: np.ndarray | None = None,
        launch_quaternion: np.ndarray | None = None,
        fx: float = 700.0,
        fy: float = 700.0,
        vo_altitude_m: float = 1.5,
        push_interval_s: float = 0.1,
        timeout_s: float = 0.25,
        tag_correction_gain: float = 0.25,
    ):
        self._client = Esp32Client(host, port, drone_id, timeout_s)
        launch_position = (
            np.array([0.0, 0.0, 1.5], dtype=np.float64)
            if launch_position is None
            else launch_position.astype(np.float64)
        )
        launch_quaternion = (
            np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
            if launch_quaternion is None
            else launch_quaternion.astype(np.float64)
        )
        self._fusion = PoseFusion(
            launch_position=launch_position,
            launch_quaternion=launch_quaternion,
            fx=fx,
            fy=fy,
            vo_altitude_m=vo_altitude_m,
            tag_correction_gain=tag_correction_gain,
        )
        self.push_interval_s = push_interval_s
        self._last_push_time = 0.0
        self._imu_failures = 0
        self._push_count = 0

    def close(self) -> None:
        self._client.close()

    def get_pose(self, timestamp: float) -> PoseEstimate:
        return PoseEstimate(
            timestamp=timestamp,
            position=self._fusion.position.copy(),
            quaternion=self._fusion.quaternion_xyzw.copy(),
        )

    def world_drone_transform(self, timestamp: float) -> np.ndarray:
        from apriltag.geometry import homogeneous_from_pose

        pose = self.get_pose(timestamp)
        return homogeneous_from_pose(pose.position, pose.quaternion)

    def update_frame(self, frame_bgr: np.ndarray, timestamp: float | None = None) -> None:
        if timestamp is None:
            timestamp = time.time()

        imu = self._client.request_sensors()
        if imu is not None:
            self._fusion.update_imu(imu.accel_m_s2, imu.gyro_rad_s, imu.timestamp)
            self._imu_failures = 0
        else:
            self._imu_failures += 1

        self._fusion.update_visual(frame_bgr, timestamp)
        self._maybe_push_to_esp32(timestamp)

    def apply_tag_correction(
        self,
        measured_world: np.ndarray,
        fused_world: np.ndarray,
        confidence: float,
    ) -> None:
        self._fusion.apply_tag_correction(measured_world, fused_world, confidence)

    def _maybe_push_to_esp32(self, timestamp: float) -> None:
        if timestamp - self._last_push_time < self.push_interval_s:
            return
        self._last_push_time = timestamp
        ok = self._client.set_state_estimate(
            self._fusion.position,
            self._fusion.velocity,
            self._fusion.orientation_euler,
            self._fusion.angular_velocity,
        )
        if ok:
            self._push_count += 1

    @property
    def stats(self) -> dict:
        snap = self._fusion.snapshot()
        snap["imu_failures"] = self._imu_failures
        snap["push_count"] = self._push_count
        return snap
