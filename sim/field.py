from __future__ import annotations

import math

import numpy as np

from .config import MissionSimConfig
from .mines import Mine
from .types import FREE, HAZARD, INFLATED


class HumanPathField:
    """2D occupancy grid for human crossing (discovered mines + 1-ft inflation)."""

    def __init__(self, config: MissionSimConfig | None = None):
        self.config = config or MissionSimConfig()
        self.grid = np.full((self.config.rows, self.config.cols), FREE, dtype=np.int8)
        self.mines: list[Mine] = []

    def world_to_cell(self, world_x: float, world_y: float) -> tuple[int, int]:
        col = int(math.floor(world_x / self.config.resolution_m))
        row = int(math.floor(world_y / self.config.resolution_m))
        col = max(0, min(self.config.cols - 1, col))
        row = max(0, min(self.config.rows - 1, row))
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = (col + 0.5) * self.config.resolution_m
        y = (row + 0.5) * self.config.resolution_m
        return x, y

    def start_cells(self) -> list[tuple[int, int]]:
        """Start edge: short side near x = margin (full width in y)."""
        margin = self.config.edge_margin_m
        col = int(margin / self.config.resolution_m)
        col = max(0, min(self.config.cols - 1, col))
        row_min = int(margin / self.config.resolution_m)
        row_max = int((self.config.field_y_m - margin) / self.config.resolution_m)
        return [(r, col) for r in range(row_min, row_max + 1)]

    def goal_cells(self) -> list[tuple[int, int]]:
        """Far edge: opposite short side near x = field_x - margin."""
        margin = self.config.edge_margin_m
        col = int((self.config.field_x_m - margin) / self.config.resolution_m)
        col = max(0, min(self.config.cols - 1, col))
        row_min = int(margin / self.config.resolution_m)
        row_max = int((self.config.field_y_m - margin) / self.config.resolution_m)
        return [(r, col) for r in range(row_min, row_max + 1)]

    def add_mines(self, mines: list[Mine]) -> None:
        self.rebuild_from_mines(mines)

    def rebuild_from_mines(self, mines: list[Mine]) -> None:
        """Inflate each mine by exactly clearance_m (default 1 ft = 0.3048 m)."""
        self.grid.fill(FREE)
        self.mines = list(mines)
        clearance = float(self.config.clearance_m)
        res = self.config.resolution_m
        r_cells = int(math.ceil(clearance / res)) + 1
        for mine in mines:
            mx, my = float(mine.world_x), float(mine.world_y)
            crow, ccol = self.world_to_cell(mx, my)
            self.grid[crow, ccol] = HAZARD
            for dr in range(-r_cells, r_cells + 1):
                for dc in range(-r_cells, r_cells + 1):
                    nr, nc = crow + dr, ccol + dc
                    if not (0 <= nr < self.config.rows and 0 <= nc < self.config.cols):
                        continue
                    cx, cy = self.cell_to_world(nr, nc)
                    if math.hypot(cx - mx, cy - my) <= clearance + 1e-9:
                        if dr == 0 and dc == 0:
                            self.grid[nr, nc] = HAZARD
                        elif self.grid[nr, nc] != HAZARD:
                            self.grid[nr, nc] = INFLATED

    def passable(self, row: int, col: int) -> bool:
        return self.grid[row, col] not in (HAZARD, INFLATED)
