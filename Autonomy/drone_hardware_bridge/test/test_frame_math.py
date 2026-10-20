import math

import pytest

from drone_hardware_bridge.frame_math import (
    quaternion_from_yaw, world_vector_to_body,
)


def test_world_velocity_is_expressed_in_odometry_child_frame_at_zero_yaw():
    assert world_vector_to_body(
        (1.0, 2.0, 3.0), quaternion_from_yaw(0.0)
    ) == pytest.approx((1.0, 2.0, 3.0))


def test_world_velocity_is_rotated_into_body_frame_at_ninety_degree_yaw():
    # A body yawed +90 degrees has +X pointing toward world +Y.
    assert world_vector_to_body(
        (1.0, 2.0, 3.0), quaternion_from_yaw(math.pi / 2.0)
    ) == pytest.approx((2.0, -1.0, 3.0))
