from __future__ import annotations

import cv2
import numpy as np


def eye_width_for_frame(width: int, height: int) -> int | None:
    if width == 2560 and height == 720:
        return 1280
    if width == 1280 and height == 720:
        return 640
    if width % 2 == 0 and width >= height * 2:
        return width // 2
    return None


def split_stereo_frame(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, int] | None:
    height, width = frame_bgr.shape[:2]
    eye_w = eye_width_for_frame(width, height)
    if eye_w is None:
        return None
    left = frame_bgr[:, :eye_w]
    right = frame_bgr[:, eye_w : eye_w * 2]
    return left, right, eye_w


def build_stereo_matcher(num_disparities: int = 144, block_size: int = 7) -> cv2.StereoSGBM:
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * block_size**2,
        P2=32 * block_size**2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def compute_depth_map(
    left_gray: np.ndarray,
    right_gray: np.ndarray,
    stereo: cv2.StereoSGBM,
    fx: float,
    baseline_m: float,
    min_depth_m: float,
    max_depth_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
    valid_mask = disparity > 0.0

    depth_map = np.zeros_like(disparity)
    depth_map[valid_mask] = (fx * baseline_m) / disparity[valid_mask]
    valid_mask &= (depth_map >= min_depth_m) & (depth_map <= max_depth_m)
    return depth_map, valid_mask


def depth_range_in_bbox(
    depth_map: np.ndarray,
    valid_mask: np.ndarray,
    x: int,
    y: int,
    w: int,
    h: int,
    min_valid_pixels: int = 30,
) -> tuple[float, float] | None:
    roi_depth = depth_map[y : y + h, x : x + w]
    roi_valid = valid_mask[y : y + h, x : x + w]
    values = roi_depth[roi_valid]
    if values.size < min_valid_pixels:
        return None
    d_near, d_far = np.percentile(values, [5, 95])
    if d_far <= d_near:
        d_far = d_near + 0.01
    return float(d_near), float(d_far)


def depth_cluster_detections(
    depth_map: np.ndarray,
    valid_mask: np.ndarray,
    min_area_px: int = 2000,
    min_depth_m: float = 0.5,
    max_depth_m: float = 8.0,
) -> list[tuple[int, int, int, int, float, float]]:
    """
    Find obstacle-like blobs from depth discontinuities (no ML).

    Returns list of (x, y, w, h, depth_near, depth_far).
    """
    mask = valid_mask.astype(np.uint8) * 255
    depth_norm = np.zeros_like(depth_map, dtype=np.uint8)
    if np.any(valid_mask):
        vals = depth_map[valid_mask]
        vmin, vmax = np.percentile(vals, [5, 95])
        vmax = max(vmax, vmin + 1e-3)
        clipped = np.clip(depth_map, vmin, vmax)
        depth_norm[valid_mask] = ((clipped[valid_mask] - vmin) / (vmax - vmin) * 255).astype(np.uint8)

    edges = cv2.Canny(depth_norm, 50, 120)
    combined = cv2.bitwise_and(mask, edges)
    combined = cv2.dilate(combined, np.ones((5, 5), np.uint8), iterations=2)

    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections: list[tuple[int, int, int, int, float, float]] = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if w * h < min_area_px:
            continue
        depth_range = depth_range_in_bbox(depth_map, valid_mask, x, y, w, h, min_valid_pixels=20)
        if depth_range is None:
            continue
        d_near, d_far = depth_range
        if d_near < min_depth_m or d_far > max_depth_m:
            continue
        detections.append((x, y, w, h, d_near, d_far))
    return detections
