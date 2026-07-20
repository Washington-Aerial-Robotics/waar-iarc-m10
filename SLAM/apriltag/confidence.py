import numpy as np


def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))


def polygon_area(pts: np.ndarray) -> float:
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def tilt_angle_degrees(rotation: np.ndarray) -> float:
    normal = rotation @ np.array([0.0, 0.0, 1.0])
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    forward = np.array([0.0, 0.0, 1.0])
    cosang = float(np.dot(normal, -forward))
    cosang = max(-1.0, min(1.0, cosang))
    return float(np.degrees(np.arccos(cosang)))


def confidence_score(
    decision_margin: float | None,
    area_px: float,
    tilt_deg: float,
) -> float:
    if decision_margin is None:
        dm_score = 0.5
    else:
        dm_score = clamp01((float(decision_margin) - 20.0) / 60.0)

    area_score = clamp01((float(area_px) - 800.0) / 8000.0)
    tilt_score = clamp01((75.0 - float(tilt_deg)) / 75.0)

    return clamp01(0.45 * dm_score + 0.40 * area_score + 0.15 * tilt_score)
