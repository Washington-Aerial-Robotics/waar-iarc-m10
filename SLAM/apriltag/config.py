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
    FUSED = "fused"


@dataclass
class PipelineConfig:
    camera_index: int = 0
    request_width: int = 1920
    request_height: int = 1080
    use_v4l2: bool = True
    force_mjpeg: bool = False
    tag_family: str = "tag36h11"
    tag_size_m: float = 0.0381

    min_confidence: float = 0.1
    enable_visualization: bool = False
    enable_csv_log: bool = True
    use_rmse_confidence: bool = True
    use_kalman_fusion: bool = True
    enable_stats_log: bool = True
    stats_interval_s: float = 5.0
    map_save_interval_frames: int = 150
    calib_file: Path = field(default_factory=lambda: DEFAULT_CALIB_FILE)
    csv_log_file: Path = field(default_factory=lambda: REPO_ROOT / "SLAM" / "mine_detections.csv")
    stats_log_file: Path = field(default_factory=lambda: REPO_ROOT / "SLAM" / "pipeline_stats.log")

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

    # Obstacle / tree detection (stereo camera required)
    enable_obstacles: bool = False
    camera_mode: str = "mono"  # "mono" | "stereo"
    obstacle_model: str = "yolov8n.pt"
    obstacle_classes: list[str] = field(default_factory=lambda: ["tree"])
    obstacle_min_confidence: float = 0.25
    obstacle_detection_mode: str = "both"  # "yolo" | "depth" | "both"
    obstacle_fusion_radius_m: float = 1.0
    obstacle_use_kalman: bool = True
    obstacle_min_depth_m: float = 0.5
    obstacle_max_depth_m: float = 8.0
    obstacle_default_height_m: float = 2.5
    obstacle_default_radius_m: float = 0.35
    stereo_baseline_m: float = 0.06
    stereo_fx: float | None = None
    obstacle_map_file: Path = field(
        default_factory=lambda: REPO_ROOT / "SLAM" / "obstacle_map.json"
    )
    shared_obstacle_map_file: Path = field(
        default_factory=lambda: REPO_ROOT / "SLAM" / "shared_obstacle_map.json"
    )

    # Fused localization (Pi IMU + VO → COM_SET_ST_EST)
    launch_position: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 1.5], dtype=np.float64)
    )
    launch_quaternion: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    )
    vo_altitude_m: float = 1.5
    pose_push_interval_s: float = 0.1
    tag_correction_gain: float = 0.25

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
            "force_mjpeg",
            "tag_family",
            "tag_size_m",
            "min_confidence",
            "enable_visualization",
            "enable_csv_log",
            "use_rmse_confidence",
            "use_kalman_fusion",
            "enable_stats_log",
            "stats_interval_s",
            "map_save_interval_frames",
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

        if "localization" in data:
            loc = data["localization"]
            for key in ("vo_altitude_m", "pose_push_interval_s", "tag_correction_gain"):
                if key in loc:
                    setattr(cfg, key, loc[key])
            if "launch_position_m" in loc:
                cfg.launch_position = np.array(loc["launch_position_m"], dtype=np.float64)
            if "launch_quaternion_xyzw" in loc:
                cfg.launch_quaternion = np.array(loc["launch_quaternion_xyzw"], dtype=np.float64)

        if "calib_file" in data:
            cfg.calib_file = Path(data["calib_file"])
            if not cfg.calib_file.is_absolute():
                cfg.calib_file = REPO_ROOT / cfg.calib_file

        if "csv_log_file" in data:
            cfg.csv_log_file = Path(data["csv_log_file"])

        if "stats_log_file" in data:
            cfg.stats_log_file = Path(data["stats_log_file"])

        if "obstacles" in data:
            obs = data["obstacles"]
            for key in (
                "enabled",
                "camera_mode",
                "model",
                "min_confidence",
                "detection_mode",
                "fusion_radius_m",
                "use_kalman",
                "min_depth_m",
                "max_depth_m",
                "default_height_m",
                "default_radius_m",
                "stereo_baseline_m",
                "stereo_fx",
            ):
                mapped = {
                    "enabled": "enable_obstacles",
                    "model": "obstacle_model",
                    "min_confidence": "obstacle_min_confidence",
                    "detection_mode": "obstacle_detection_mode",
                    "fusion_radius_m": "obstacle_fusion_radius_m",
                    "use_kalman": "obstacle_use_kalman",
                    "min_depth_m": "obstacle_min_depth_m",
                    "max_depth_m": "obstacle_max_depth_m",
                    "default_height_m": "obstacle_default_height_m",
                    "default_radius_m": "obstacle_default_radius_m",
                }.get(key, key)
                if key in obs:
                    setattr(cfg, mapped, obs[key])
            if "target_classes" in obs:
                cfg.obstacle_classes = list(obs["target_classes"])
            if "map_file" in obs:
                cfg.obstacle_map_file = Path(obs["map_file"])
            if "shared_map_file" in obs:
                cfg.shared_obstacle_map_file = Path(obs["shared_map_file"])

        for path_key in ("obstacle_map_file", "shared_obstacle_map_file"):
            path_val = getattr(cfg, path_key)
            if not path_val.is_absolute():
                setattr(cfg, path_key, REPO_ROOT / path_val)

        return cfg
