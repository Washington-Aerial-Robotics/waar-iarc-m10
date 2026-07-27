from __future__ import annotations

import heapq
import math
from typing import Optional

from .field import HumanPathField


def astar_human_path(field: HumanPathField) -> Optional[list[tuple[int, int]]]:
    """
    Lowest-cost path from any start-line cell to any goal-line cell.
    Cells are (row, col). 8-connected, uniform step cost.
    """
    start_cells = [c for c in field.start_cells() if field.passable(c[0], c[1])]
    goal_set = set(field.goal_cells())
    if not start_cells or not goal_set:
        return None

    goal_cols = [c[1] for c in goal_set]
    goal_col_hint = sum(goal_cols) / len(goal_cols)

    def h(row: int, col: int) -> float:
        return abs(col - goal_col_hint)

    open_heap: list[tuple[float, float, int, int]] = []
    g_score: dict[tuple[int, int], float] = {}
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {}

    for row, col in start_cells:
        g_score[(row, col)] = 0.0
        came_from[(row, col)] = None
        heapq.heappush(open_heap, (h(row, col), 0.0, row, col))

    while open_heap:
        _, g, row, col = heapq.heappop(open_heap)
        if g > g_score.get((row, col), 1e18):
            continue
        if (row, col) in goal_set:
            return _reconstruct(came_from, (row, col))

        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nr, nc = row + dr, col + dc
            if not (0 <= nr < field.config.rows and 0 <= nc < field.config.cols):
                continue
            if not field.passable(nr, nc):
                continue
            ng = g + math.hypot(dr, dc)
            if ng < g_score.get((nr, nc), 1e18):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = (row, col)
                heapq.heappush(open_heap, (ng + h(nr, nc), ng, nr, nc))

    return None


def path_length_m(field: HumanPathField, path: list[tuple[int, int]]) -> float:
    if len(path) < 2:
        return 0.0
    total = 0.0
    res = field.config.resolution_m
    for (r0, c0), (r1, c1) in zip(path[:-1], path[1:]):
        total += math.hypot((c1 - c0) * res, (r1 - r0) * res)
    return total


def _reconstruct(
    came_from: dict[tuple[int, int], tuple[int, int] | None],
    end: tuple[int, int],
) -> list[tuple[int, int]]:
    path: list[tuple[int, int]] = []
    node: tuple[int, int] | None = end
    while node is not None:
        path.append(node)
        node = came_from[node]
    path.reverse()
    return path
