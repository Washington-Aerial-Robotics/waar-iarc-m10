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

    field_x: float = 91.44
    field_y: float = 24.38
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

    camera_mode: str = "mono"  # "mono" | "stereo" (side-by-side; left eye for tags)
    stereo_baseline_m: float = 0.06
    stereo_fx: float | None = None

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

    # Classical PFM-1 shape branch (parallel to AprilTags)
    enable_shape_detection: bool = True
    min_shape_confidence: float = 0.35
    shape_template_path: Path | None = None
    pfm_physical_span_m: float = 0.12  # TODO: confirm wing span from IARC Resource Addendum
    shape_max_match_distance: float = 0.45
    shape_dedupe_radius_m: float = 0.5
    shape_fusion_radius_m: float = 0.5
    shape_canny_low: int = 40
    shape_canny_high: int = 120
    shape_min_contour_area_px: float = 400.0
    shape_max_contour_area_px: float = 80000.0
    ground_z_m: float = 0.0

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
            "camera_mode",
            "stereo_baseline_m",
            "stereo_fx",
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

        # Legacy configs used an "obstacles" block only for camera_mode
        if "obstacles" in data and "camera_mode" in data["obstacles"]:
            cfg.camera_mode = data["obstacles"]["camera_mode"]
            obs = data["obstacles"]
            if "stereo_baseline_m" in obs:
                cfg.stereo_baseline_m = obs["stereo_baseline_m"]
            if "stereo_fx" in obs:
                cfg.stereo_fx = obs["stereo_fx"]

        if "mine_shape" in data:
            ms = data["mine_shape"]
            for key in (
                "enable_shape_detection",
                "min_shape_confidence",
                "pfm_physical_span_m",
                "shape_max_match_distance",
                "shape_dedupe_radius_m",
                "shape_fusion_radius_m",
                "shape_canny_low",
                "shape_canny_high",
                "shape_min_contour_area_px",
                "shape_max_contour_area_px",
                "ground_z_m",
            ):
                if key in ms:
                    setattr(cfg, key, ms[key])
            if "template_path" in ms and ms["template_path"]:
                cfg.shape_template_path = Path(ms["template_path"])
                if not cfg.shape_template_path.is_absolute():
                    cfg.shape_template_path = REPO_ROOT / cfg.shape_template_path

        return cfg
