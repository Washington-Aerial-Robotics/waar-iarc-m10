from __future__ import annotations

import math

import numpy as np

from .models import ShapeMineCandidate


def _xy_dist(a: np.ndarray, b: np.ndarray) -> float:
    return float(math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])))


def filter_shapes_away_from_tags(
    candidates: list[ShapeMineCandidate],
    tag_world_positions: list[np.ndarray],
    *,
    radius_m: float,
) -> list[ShapeMineCandidate]:
    """Drop shape hits that overlap a tag-decoded mine in XY (tag wins)."""
    if not tag_world_positions:
        return candidates
    kept: list[ShapeMineCandidate] = []
    for cand in candidates:
        if any(_xy_dist(cand.world_position, tp) < radius_m for tp in tag_world_positions):
            continue
        kept.append(cand)
    return kept


def remove_shapes_near_tag_world(shape_registry, tag_world_position: np.ndarray, *, radius_m: float) -> int:
    """Remove pending shape-only mines superseded by a tag fix."""
    return shape_registry.remove_near(tag_world_position, radius_m)
