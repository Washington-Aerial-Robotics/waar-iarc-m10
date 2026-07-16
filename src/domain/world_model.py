from __future__ import annotations

import numpy as np

from domain.types import K, FREE, UNKNOWN


class WorldModel:
    """
    Shared belief state updated by the drone as it explores.

    Wraps map2 (visited block grid) and map3 (detected fine grid).
    start_line and goal_line are full vertical lines (margin of K cells
    from the top/bottom edges) on the left and right sides of the arena,
    pre-marked FREE so A* can always reach them. The drone may cross the
    minefield starting and ending anywhere along these lines.
    """

    def __init__(self, block_cols: int, block_rows: int) -> None:
        self.block_cols = block_cols
        self.block_rows = block_rows
        FC = block_cols * K
        FR = block_rows * K

        self.visited  = np.zeros((block_cols, block_rows), dtype=bool)   # map2
        self.detected = np.full((FC, FR), UNKNOWN, dtype=np.int8)        # map3

        start_x = K
        goal_x  = FC - K - 1

        self.start_line = [(start_x, y) for y in range(K, FR - K)]
        self.goal_line  = [(goal_x, y) for y in range(K, FR - K)]

        # Backward-compatible single-point aliases (midpoint of each line).
        self.start_fine = self.start_line[len(self.start_line) // 2]
        self.goal_fine  = self.goal_line[len(self.goal_line) // 2]

        for sx, sy in self.start_line:
            self.detected[sx, sy] = FREE
        for gx, gy in self.goal_line:
            self.detected[gx, gy] = FREE

    @property
    def fine_cols(self) -> int:
        return self.block_cols * K

    @property
    def fine_rows(self) -> int:
        return self.block_rows * K
