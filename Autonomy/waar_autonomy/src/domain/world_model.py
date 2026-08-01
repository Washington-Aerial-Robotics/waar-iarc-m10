from __future__ import annotations

import numpy as np

from domain.types import K, FREE, UNKNOWN


class WorldModel:
    """
    Shared belief state updated by the drone as it explores.

    Wraps map2 (visited block grid) and map3 (detected fine grid).
    start_fine and goal_fine are pre-marked FREE so A* can always reach them.
    """

    def __init__(self, block_cols: int, block_rows: int) -> None:
        self.block_cols = block_cols
        self.block_rows = block_rows
        FC = block_cols * K
        FR = block_rows * K

        self.visited  = np.zeros((block_cols, block_rows), dtype=bool)   # map2
        self.detected = np.full((FC, FR), UNKNOWN, dtype=np.int8)        # map3

        self.start_fine: tuple[int, int] = (K,        FR - K - 1)
        self.goal_fine:  tuple[int, int] = (FC - K - 1, K)

        sx, sy = self.start_fine
        gx, gy = self.goal_fine
        self.detected[sx, sy] = FREE
        self.detected[gx, gy] = FREE

    @property
    def fine_cols(self) -> int:
        return self.block_cols * K

    @property
    def fine_rows(self) -> int:
        return self.block_rows * K
