from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CALIB_FILE = REPO_ROOT / "apriltags" / "camera_calib.npz"
DEFAULT_CONFIG_FILE = REPO_ROOT / "SLAM" / "pipeline_config.json"


class PoseSource(str, Enum):
    STUB = "stub"
    ESP32 = "esp32"


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
    use_rmse_confidence: bool = True
    use_kalman_fusion: bool = True
    csv_log_file: Path = field(default_factory=lambda: REPO_ROOT / "SLAM" / "mine_detections.csv")

    field_x: float = 94.0
    field_y: float = 12.0
    map_resolution: float = 0.2

    pose_source: PoseSource = PoseSource.STUB
    esp32_host: str = "192.168.1.100"
    esp32_port: int = 23
    esp32_drone_id: int = 65

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

    @classmethod
    def from_json(cls, path: Path | None = None) -> "PipelineConfig":
        path = path or DEFAULT_CONFIG_FILE
        if not path.exists():
            return cls()

        data = json.loads(path.read_text(encoding="utf-8"))
        cfg = cls()

        for key in (
            "camera_index",
            "request_width",
            "request_height",
            "use_v4l2",
            "tag_family",
            "tag_size_m",
            "min_confidence",
            "enable_visualization",
            "enable_csv_log",
            "use_rmse_confidence",
            "use_kalman_fusion",
            "field_x",
            "field_y",
            "map_resolution",
            "esp32_host",
            "esp32_port",
            "esp32_drone_id",
        ):
            if key in data:
                setattr(cfg, key, data[key])

        if "pose_source" in data:
            cfg.pose_source = PoseSource(data["pose_source"])

        if "drone_camera" in data:
            cam = data["drone_camera"]
            cfg.drone_camera_position = np.array(cam.get("position_m", [0, 0, 0]), dtype=np.float64)
            cfg.drone_camera_quaternion = np.array(
                cam.get("quaternion_xyzw", [0, 0, 0, 1]), dtype=np.float64
            )

        if "stub_drone" in data:
            stub = data["stub_drone"]
            cfg.stub_drone_position = np.array(stub.get("position_m", [0, 0, 1.5]), dtype=np.float64)
            cfg.stub_drone_quaternion = np.array(
                stub.get("quaternion_xyzw", [0, 0, 0, 1]), dtype=np.float64
            )

        return cfg
