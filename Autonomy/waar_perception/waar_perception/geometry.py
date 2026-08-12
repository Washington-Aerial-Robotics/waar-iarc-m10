"""Pure geometry and filtering helpers used by the AprilTag ROS node.

This module intentionally has no ROS or OpenCV imports so it can be tested on a
developer laptop and in CI without camera hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Tuple


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def normalize_quaternion(
    quaternion_xyzw: Iterable[float],
) -> Tuple[float, float, float, float]:
    """Return a normalized quaternion, rejecting invalid or zero input."""
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    if not _finite((x, y, z, w)):
        raise ValueError("quaternion contains a non-finite value")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    return x / norm, y / norm, z / norm, w / norm


def rotate_vector(
    vector_xyz: Iterable[float], quaternion_xyzw: Iterable[float]
) -> Tuple[float, float, float]:
    """Rotate ``vector_xyz`` by ``quaternion_xyzw``."""
    vx, vy, vz = (float(value) for value in vector_xyz)
    if not _finite((vx, vy, vz)):
        raise ValueError("vector contains a non-finite value")
    qx, qy, qz, qw = normalize_quaternion(quaternion_xyzw)

    # Efficient quaternion-vector multiplication: v' = v + 2w(q x v)
    # + 2(q x (q x v)).
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def transform_point(
    point_xyz: Iterable[float],
    translation_xyz: Iterable[float],
    quaternion_xyzw: Iterable[float],
) -> Tuple[float, float, float]:
    """Apply a rigid transform to a 3-D point."""
    px, py, pz = rotate_vector(point_xyz, quaternion_xyzw)
    tx, ty, tz = (float(value) for value in translation_xyz)
    if not _finite((tx, ty, tz)):
        raise ValueError("translation contains a non-finite value")
    return px + tx, py + ty, pz + tz


def detection_confidence(
    reprojection_rmse_px: float,
    pixel_area: float,
    minimum_pixel_area: float,
    error_scale_px: float,
) -> float:
    """Map tag geometry quality to a bounded confidence in ``[0, 1]``.

    Small detections and large reprojection error are both penalized.  This is
    deliberately conservative: the MAS coordinator can request a close-range
    verification instead of treating a weak first sighting as confirmed.
    """
    values = (reprojection_rmse_px, pixel_area, minimum_pixel_area, error_scale_px)
    if not _finite(values):
        return 0.0
    if reprojection_rmse_px < 0.0 or pixel_area <= 0.0:
        return 0.0
    if minimum_pixel_area <= 0.0 or error_scale_px <= 0.0:
        raise ValueError("confidence scales must be positive")
    area_score = min(1.0, pixel_area / minimum_pixel_area)
    error_score = math.exp(-reprojection_rmse_px / error_scale_px)
    return max(0.0, min(1.0, area_score * error_score))


def inside_arena(
    x: float,
    y: float,
    bounds: Iterable[float],
) -> bool:
    """Return whether a finite point is inside inclusive XY arena bounds."""
    x_min, x_max, y_min, y_max = (float(value) for value in bounds)
    values = (x, y, x_min, x_max, y_min, y_max)
    if not _finite(values):
        return False
    if x_min > x_max or y_min > y_max:
        raise ValueError("arena bounds are inverted")
    return x_min <= x <= x_max and y_min <= y <= y_max


@dataclass
class MineTrack:
    """Confidence-weighted position estimate for one stable AprilTag ID."""

    mine_id: str
    x: float
    y: float
    confidence: float
    observations: int = 1
    sequence: int = 1

    @property
    def verification_ready(self) -> bool:
        """Require repeat observations before a track can be confirmed."""
        return self.observations >= 3

    def update(self, x: float, y: float, confidence: float) -> bool:
        """Fuse one observation and return whether a publish is warranted."""
        if not _finite((x, y, confidence)) or not 0.0 <= confidence <= 1.0:
            raise ValueError("invalid mine observation")
        old_x, old_y, old_confidence = self.x, self.y, self.confidence
        old_weight = max(0.05, self.confidence) * self.observations
        new_weight = max(0.05, confidence)
        total = old_weight + new_weight
        self.x = (self.x * old_weight + x * new_weight) / total
        self.y = (self.y * old_weight + y * new_weight) / total
        self.confidence = max(self.confidence, confidence)
        self.observations += 1
        changed = (
            math.hypot(self.x - old_x, self.y - old_y) >= 0.02
            or self.confidence >= old_confidence + 0.02
        )
        if changed:
            self.sequence += 1
        return changed
