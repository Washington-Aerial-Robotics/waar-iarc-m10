from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CALIB_FILE = REPO_ROOT / "apriltags" / "camera_calib.npz"


@dataclass
class PipelineConfig:
    camera_index: int = 0
    request_width: int = 1920
    request_height: int = 1080
    use_v4l2: bool = True

    calib_file: Path = field(default_factory=lambda: DEFAULT_CALIB_FILE)
    tag_family: str = "tag36h11"
    tag_size_m: float = 0.0381

    min_confidence: float = 0.1
    enable_visualization: bool = False
    enable_csv_log: bool = True
    csv_log_file: Path = field(default_factory=lambda: REPO_ROOT / "SLAM" / "mine_detections.csv")

    field_x: float = 94.0
    field_y: float = 12.0
    map_resolution: float = 0.2

    drone_camera_position: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float64)
    )
    drone_camera_quaternion: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    )

    stub_drone_position: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 1.5], dtype=np.float64)
    )
    stub_drone_quaternion: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    )
