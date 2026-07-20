from __future__ import annotations

import socket
import struct
import time
from abc import ABC, abstractmethod

import numpy as np

from .config import PipelineConfig, PoseSource
from .geometry import homogeneous_from_pose
from .models import PoseEstimate


class PoseProvider(ABC):
    @abstractmethod
    def get_pose(self, timestamp: float) -> PoseEstimate:
        raise NotImplementedError

    def world_drone_transform(self, timestamp: float) -> np.ndarray:
        pose = self.get_pose(timestamp)
        return homogeneous_from_pose(pose.position, pose.quaternion)


class StubPoseProvider(PoseProvider):
    """Fixed drone pose for integration testing before real localization is connected."""

    def __init__(self, position: np.ndarray, quaternion_xyzw: np.ndarray):
        self.position = position.astype(np.float64).reshape(3)
        self.quaternion = quaternion_xyzw.astype(np.float64).reshape(4)

    def get_pose(self, timestamp: float) -> PoseEstimate:
        return PoseEstimate(
            timestamp=timestamp,
            position=self.position.copy(),
            quaternion=self.quaternion.copy(),
        )

    def set_pose(self, position: np.ndarray, quaternion_xyzw: np.ndarray) -> None:
        self.position = position.astype(np.float64).reshape(3)
        self.quaternion = quaternion_xyzw.astype(np.float64).reshape(4)


class Esp32PoseProvider(PoseProvider):
    """
    Request drone position from ESP32 over TCP.

    Sends COM_REQUEST_POS (0x63) and parses COM_REPLY_POS (0x23) coordinate payload.
    Attitude is not yet available from this message — identity quaternion is used.
    """

    COM_REQUEST_POS = 0x63
    COM_REPLY_POS = 0x23
    APP_DEVICE_ID = 0x47

    def __init__(
        self,
        host: str,
        port: int = 23,
        drone_id: int = 65,
        timeout_s: float = 0.25,
    ):
        self.host = host
        self.port = port
        self.drone_id = drone_id & 0xFF
        self.timeout_s = timeout_s
        self._last_pose = PoseEstimate(
            timestamp=time.time(),
            position=np.array([0.0, 0.0, 1.5], dtype=np.float64),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        )

    def get_pose(self, timestamp: float) -> PoseEstimate:
        packet = bytes(
            [
                self.drone_id,
                self.APP_DEVICE_ID,
                self.COM_REQUEST_POS,
                int(timestamp * 1000) & 0xFF,
            ]
        )
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout_s) as sock:
                sock.sendall(packet)
                sock.settimeout(self.timeout_s)
                response = sock.recv(32)
        except OSError:
            return self._last_pose

        if len(response) < 20:
            return self._last_pose

        if response[2] != self.COM_REPLY_POS:
            return self._last_pose

        x, y, z, _stdev = struct.unpack_from("<4f", response, 4)
        self._last_pose = PoseEstimate(
            timestamp=timestamp,
            position=np.array([x, y, z], dtype=np.float64),
            quaternion=self._last_pose.quaternion.copy(),
        )
        return self._last_pose


def create_pose_provider(config: PipelineConfig) -> PoseProvider:
    if config.pose_source == PoseSource.ESP32:
        return Esp32PoseProvider(
            host=config.esp32_host,
            port=config.esp32_port,
            drone_id=config.esp32_drone_id,
        )
    return StubPoseProvider(
        position=config.stub_drone_position,
        quaternion_xyzw=config.stub_drone_quaternion,
    )
