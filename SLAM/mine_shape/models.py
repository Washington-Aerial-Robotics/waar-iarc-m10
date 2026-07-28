from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ShapeMineCandidate:
    """Single-frame PFM-1 shape hypothesis."""

    timestamp: float
    center_px: tuple[float, float]
    confidence: float
    world_position: np.ndarray
    tag_id: None = None
    match_distance: float = 0.0
    contour_area_px: float = 0.0
    apparent_span_px: float = 0.0
