from __future__ import annotations

import numpy as np


def backproject_pixel(u: float, v: float, depth_m: float, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return np.array([x, y, z], dtype=np.float64)


def bbox_corners_camera(
    x: int,
    y: int,
    w: int,
    h: int,
    depth_near_m: float,
    depth_far_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    """Eight corners of an axis-aligned box from a 2D bbox and depth range."""
    pixel_corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
    corners = [
        backproject_pixel(u, v, depth_m, fx, fy, cx, cy)
        for depth_m in (depth_near_m, depth_far_m)
        for (u, v) in pixel_corners
    ]
    return np.array(corners, dtype=np.float64)


def camera_points_to_world(
    points_camera: np.ndarray,
    world_drone_transform: np.ndarray,
    drone_camera_transform: np.ndarray,
) -> np.ndarray:
    """Transform Nx3 camera-frame points to world frame."""
    world_camera = world_drone_transform @ drone_camera_transform
    rotation = world_camera[:3, :3]
    translation = world_camera[:3, 3]
    return (rotation @ points_camera.T).T + translation
