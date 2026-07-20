from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class AprilTagDetection:
    """Camera-frame AprilTag measurement."""

    timestamp: float
    tag_id: int
    translation_camera: np.ndarray
    rotation_camera: np.ndarray
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    confidence: float
    decision_margin: float | None
    area_px: float
    corners_px: np.ndarray


@dataclass
class PoseEstimate:
    """Drone pose in world frame."""

    timestamp: float
    position: np.ndarray
    quaternion: np.ndarray


@dataclass
class FusedMine:
    """Fused world estimate for one mine (tag)."""

    tag_id: int
    first_seen: float
    last_seen: float
    observation_count: int
    world_position: np.ndarray
    confidence: float
    world_rotation: np.ndarray | None = field(default=None)
