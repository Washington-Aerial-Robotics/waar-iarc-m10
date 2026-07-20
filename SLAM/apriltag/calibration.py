from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: tuple[int, int] | None
    camera_params: tuple[float, float, float, float]


def load_calibration(calib_file: Path) -> CameraCalibration:
    if not calib_file.exists():
        raise FileNotFoundError(f"Camera calibration not found: {calib_file}")

    calib = np.load(calib_file)
    camera_matrix = calib["camera_matrix"]
    dist_coeffs = calib["dist_coeffs"]

    image_size = None
    if "image_size" in calib:
        size = calib["image_size"]
        image_size = (int(size[0]), int(size[1]))

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])

    return CameraCalibration(
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        camera_params=(fx, fy, cx, cy),
    )
