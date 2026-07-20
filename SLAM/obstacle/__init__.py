from __future__ import annotations

import json
from pathlib import Path

from .detector import ObstacleDetector, ObstacleRegistry
from .geometry import bbox_corners_camera, camera_points_to_world
from .models import FusedObstacle, ObstacleDetection
from .stereo import (
    build_stereo_matcher,
    compute_depth_map,
    depth_cluster_detections,
    depth_range_in_bbox,
    eye_width_for_frame,
    split_stereo_frame,
)

__all__ = [
    "FusedObstacle",
    "ObstacleDetection",
    "ObstacleDetector",
    "ObstacleRegistry",
    "bbox_corners_camera",
    "build_stereo_matcher",
    "camera_points_to_world",
    "compute_depth_map",
    "depth_cluster_detections",
    "depth_range_in_bbox",
    "eye_width_for_frame",
    "export_obstacle_map",
    "import_obstacle_map",
    "split_stereo_frame",
]


def export_obstacle_map(registry: ObstacleRegistry, path: Path) -> None:
    path.write_text(json.dumps(registry.export_records(), indent=2), encoding="utf-8")


def import_obstacle_map(registry: ObstacleRegistry, path: Path) -> int:
    if not path.exists():
        return 0
    records = json.loads(path.read_text(encoding="utf-8"))
    return registry.import_records(records)
