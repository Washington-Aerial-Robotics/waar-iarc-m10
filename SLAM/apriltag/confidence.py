import cv2
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


def confidence_from_metrics(
    decision_margin: float | None,
    area_px: float,
    reproj_rmse_px: float,
    tilt_deg: float,
) -> float:
    if decision_margin is None:
        dm_score = 0.5
    else:
        dm_score = clamp01((float(decision_margin) - 20.0) / 60.0)

    area_score = clamp01((float(area_px) - 800.0) / 8000.0)
    reproj_score = clamp01((6.0 - float(reproj_rmse_px)) / 5.5)
    tilt_score = clamp01((75.0 - float(tilt_deg)) / 75.0)

    return clamp01(
        0.40 * dm_score + 0.25 * area_score + 0.25 * reproj_score + 0.10 * tilt_score
    )


def pose_reprojection_rmse_px(
    det_corners_px: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
    tag_size_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> float:
    half = tag_size_m / 2.0
    tag_obj = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float32,
    )
    rvec, _ = cv2.Rodrigues(rotation.astype(np.float64))
    tvec = translation.reshape(3, 1).astype(np.float64)
    proj, _ = cv2.projectPoints(
        tag_obj, rvec, tvec, camera_matrix.astype(np.float64), dist_coeffs.astype(np.float64)
    )
    proj = proj.reshape(-1, 2).astype(np.float64)
    corners = det_corners_px.astype(np.float64)
    return float(np.sqrt(np.mean(np.sum((proj - corners) ** 2, axis=1))))
