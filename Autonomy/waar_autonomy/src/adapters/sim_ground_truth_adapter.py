from __future__ import annotations

from infrastructure.env.ground_truth_map import GroundTruthMap


class SimGroundTruthAdapter:
    """Adapter: wraps GroundTruthMap to satisfy GroundTruthPort."""

    def __init__(self, gt_map: GroundTruthMap) -> None:
        self._map = gt_map

    def is_hazard(self, fx: int, fy: int) -> bool:
        return self._map.is_hazard(fx, fy)
