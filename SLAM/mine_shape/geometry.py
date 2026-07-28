from __future__ import annotations

import numpy as np

from apriltag.geometry import tag_pose_to_world


def estimate_depth_from_span_px(
    apparent_span_px: float,
    fx: float,
    physical_span_m: float,
) -> float:
    """Pinhole: depth ≈ fx * object_size / pixel_size."""
    span = max(float(apparent_span_px), 1.0)
    return float(fx * physical_span_m / span)


def pixel_to_camera_ray(
    u: float,
    v: float,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> np.ndarray:
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return np.array([x, y, z], dtype=np.float64)


def shape_center_to_world(
    center_px: tuple[float, float],
    apparent_span_px: float,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    physical_span_m: float,
    world_drone_transform: np.ndarray,
    drone_camera_transform: np.ndarray,
    ground_z_m: float = 0.0,
) -> np.ndarray:
    """Map image center + known PFM-1 span to world XYZ (flat mine on ground plane)."""
    depth = estimate_depth_from_span_px(apparent_span_px, fx, physical_span_m)
    translation_camera = pixel_to_camera_ray(center_px[0], center_px[1], depth, fx, fy, cx, cy)
    rotation_camera = np.eye(3, dtype=np.float64)

    world_position, _ = tag_pose_to_world(
        translation_camera,
        rotation_camera,
        world_drone_transform,
        drone_camera_transform,
    )

    if world_position[2] > ground_z_m + 0.05:
        drone_z = float(world_drone_transform[2, 3])
        cam_offset_z = float(drone_camera_transform[2, 3])
        cam_z = drone_z + cam_offset_z
        if cam_z > ground_z_m + 0.1:
            t = (cam_z - ground_z_m) / max(cam_z - float(world_position[2]), 0.1)
            t = max(0.0, min(1.0, t))
            blend = world_drone_transform[:3, 3] * (1 - t) + world_position * t
            world_position = blend.copy()
            world_position[2] = ground_z_m

    return world_position
