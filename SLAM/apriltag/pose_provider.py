from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from .geometry import homogeneous_from_pose
from .models import PoseEstimate


class PoseProvider(ABC):
    @abstractmethod
    def get_pose(self, timestamp: float) -> PoseEstimate:
        raise NotImplementedError


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

    def world_drone_transform(self, timestamp: float) -> np.ndarray:
        pose = self.get_pose(timestamp)
        return homogeneous_from_pose(pose.position, pose.quaternion)
