from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ObstacleDetection:
    """Single-frame obstacle measurement in camera frame."""

    timestamp: float
    label: str
    confidence: float
    bbox_xywh: tuple[int, int, int, int]
    depth_near_m: float
    depth_far_m: float
    corners_camera: np.ndarray  # 8x3 box corners in camera frame


@dataclass
class FusedObstacle:
    """Fused world estimate for one obstacle (tree, pole, etc.)."""

    obstacle_id: int
    label: str
    first_seen: float
    last_seen: float
    observation_count: int
    world_position: np.ndarray
    confidence: float
    height_m: float = 2.0
    radius_m: float = 0.3
