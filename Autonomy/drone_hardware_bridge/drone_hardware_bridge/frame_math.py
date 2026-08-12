"""Small rigid-transform helpers shared by the ROS node and pure tests."""

from __future__ import annotations

import math
from typing import Tuple

from .protocol import quaternion_multiply


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


def normalize_quaternion(q: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in q)


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    q = normalize_quaternion(q)
    conjugate = (-q[0], -q[1], -q[2], q[3])
    pure = (vector[0], vector[1], vector[2], 0.0)
    result = quaternion_multiply(quaternion_multiply(q, pure), conjugate)
    return result[0], result[1], result[2]


def world_vector_to_body(
    vector_world: Vector3, quaternion_world_from_body: Quaternion
) -> Vector3:
    """Express a world-frame vector in the ROS body frame.

    ``nav_msgs/Odometry.twist`` is expressed in ``child_frame_id``. ESP32
    telemetry carries velocity in local ENU, so publishing it directly would
    silently become wrong whenever the aircraft has non-zero yaw.
    """
    q = normalize_quaternion(quaternion_world_from_body)
    body_from_world = (-q[0], -q[1], -q[2], q[3])
    return rotate_vector(body_from_world, vector_world)


def transform_pose(
    position: Vector3,
    orientation: Quaternion,
    translation: Vector3,
    rotation: Quaternion,
) -> tuple[Vector3, Quaternion]:
    rotated = rotate_vector(rotation, position)
    target_position = tuple(rotated[index] + translation[index] for index in range(3))
    target_orientation = normalize_quaternion(quaternion_multiply(rotation, orientation))
    return target_position, target_orientation


def quaternion_from_yaw(yaw: float) -> Quaternion:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
