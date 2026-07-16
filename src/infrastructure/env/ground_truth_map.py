from __future__ import annotations

import numpy as np


class GroundTruthMap:
    """Hidden ground-truth hazard positions (map1). Never read by planners."""

    def __init__(self, fine_cols: int, fine_rows: int) -> None:
        self.fine_cols = fine_cols
        self.fine_rows = fine_rows
        self.data = np.zeros((fine_cols, fine_rows), dtype=bool)

    def is_hazard(self, fx: int, fy: int) -> bool:
        return bool(self.data[fx, fy])
