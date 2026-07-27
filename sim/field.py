from __future__ import annotations

import math

import numpy as np

from .config import MissionSimConfig
from .mines import Mine
from .types import FREE, HAZARD, INFLATED


class HumanPathField:
    """2D occupancy grid for human crossing (mines + inflation only)."""

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
        margin = self.config.edge_margin_m
        col = int(margin / self.config.resolution_m)
        col = max(0, min(self.config.cols - 1, col))
        row_min = int(margin / self.config.resolution_m)
        row_max = int((self.config.field_y_m - margin) / self.config.resolution_m)
        return [(r, col) for r in range(row_min, row_max + 1)]

    def goal_cells(self) -> list[tuple[int, int]]:
        margin = self.config.edge_margin_m
        col = int((self.config.field_x_m - margin) / self.config.resolution_m)
        col = max(0, min(self.config.cols - 1, col))
        row_min = int(margin / self.config.resolution_m)
        row_max = int((self.config.field_y_m - margin) / self.config.resolution_m)
        return [(r, col) for r in range(row_min, row_max + 1)]

    def add_mines(self, mines: list[Mine]) -> None:
        self.rebuild_from_mines(mines)

    def rebuild_from_mines(self, mines: list[Mine]) -> None:
        self.grid.fill(FREE)
        self.mines = list(mines)
        for mine in mines:
            row, col = self.world_to_cell(mine.world_x, mine.world_y)
            self._inflate_mine(row, col)

    def _inflate_mine(self, row: int, col: int) -> None:
        r_cells = int(math.ceil(self.config.clearance_m / self.config.resolution_m))
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if dr * dr + dc * dc > r_cells * r_cells:
                    continue
                nr, nc = row + dr, col + dc
                if not (0 <= nr < self.config.rows and 0 <= nc < self.config.cols):
                    continue
                if dr == 0 and dc == 0:
                    self.grid[nr, nc] = HAZARD
                elif self.grid[nr, nc] != HAZARD:
                    self.grid[nr, nc] = INFLATED

    def passable(self, row: int, col: int) -> bool:
        return self.grid[row, col] not in (HAZARD, INFLATED)
