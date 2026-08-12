import math

import pytest

from waar_perception.geometry import (
    MineTrack,
    detection_confidence,
    inside_arena,
    normalize_quaternion,
    rotate_vector,
    transform_point,
)


def test_normalize_rejects_zero_and_nonfinite():
    with pytest.raises(ValueError):
        normalize_quaternion((0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        normalize_quaternion((0.0, 0.0, math.inf, 1.0))


def test_rotate_vector_quarter_turn_about_z():
    q = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    x, y, z = rotate_vector((1.0, 0.0, 0.0), q)
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(1.0, abs=1e-9)
    assert z == pytest.approx(0.0, abs=1e-9)


def test_transform_point_rotates_then_translates():
    q = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    assert transform_point((1.0, 0.0, 0.0), (2.0, 3.0, 4.0), q) == pytest.approx(
        (2.0, 4.0, 4.0)
    )


def test_detection_confidence_penalizes_error_and_tiny_tags():
    good = detection_confidence(0.25, 900.0, 225.0, 4.0)
    blurry = detection_confidence(8.0, 900.0, 225.0, 4.0)
    tiny = detection_confidence(0.25, 25.0, 225.0, 4.0)
    assert 0.8 < good <= 1.0
    assert blurry < good
    assert tiny < good
    assert detection_confidence(math.nan, 100.0, 100.0, 4.0) == 0.0


def test_mine_track_uses_stable_id_and_monotonic_sequence():
    track = MineTrack("tag_7", 1.0, 2.0, 0.4)
    assert track.sequence == 1
    changed = track.update(1.2, 2.0, 0.8)
    assert changed
    assert track.mine_id == "tag_7"
    assert track.sequence == 2
    assert 1.0 < track.x < 1.2
    assert track.confidence == 0.8


def test_mine_track_rejects_invalid_observation():
    track = MineTrack("tag_1", 0.0, 0.0, 0.5)
    with pytest.raises(ValueError):
        track.update(0.0, 0.0, 1.1)


def test_arena_filter_fails_closed():
    bounds = (0.0, 10.0, -2.0, 2.0)
    assert inside_arena(0.0, 2.0, bounds)
    assert not inside_arena(-0.001, 0.0, bounds)
    assert not inside_arena(math.nan, 0.0, bounds)
    with pytest.raises(ValueError):
        inside_arena(1.0, 1.0, (10.0, 0.0, -2.0, 2.0))


def test_track_requires_three_observations_before_verification():
    track = MineTrack("tag_4", 1.0, 1.0, 0.8)
    assert not track.verification_ready
    track.update(1.0, 1.0, 0.8)
    assert not track.verification_ready
    track.update(1.0, 1.0, 0.8)
    assert track.verification_ready
