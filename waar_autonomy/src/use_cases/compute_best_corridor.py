"""UC2 — Compute Best Corridor.

Runs A* on the fine-grid detected map (map3) from start_fine to goal_fine.
UNKNOWN cells incur a configurable cost (optimistic planning).
HAZARD and INFLATED cells are impassable.
"""
from __future__ import annotations

import heapq
import math
from typing import Optional

from domain.types import UNKNOWN, HAZARD, INFLATED
from domain.world_model import WorldModel


def compute_best_corridor(
    world: WorldModel,
    unknown_cost: float,
) -> Optional[list[tuple[int, int]]]:
    """Return the lowest-cost fine-cell path start→goal, or None if unreachable."""
    FC, FR = world.fine_cols, world.fine_rows
    sx, sy = world.start_fine
    gx, gy = world.goal_fine
    detected = world.detected

    def passable(x: int, y: int) -> bool:
        return detected[x, y] not in (HAZARD, INFLATED)

    def cell_cost(x: int, y: int) -> float:
        return unknown_cost if detected[x, y] == UNKNOWN else 1.0

    def h(x: int, y: int) -> float:
        return math.hypot(x - gx, y - gy)

    open_heap: list = []
    g_score:   dict = {}
    came_from: dict = {}

    g_score[(sx, sy)] = 0.0
    came_from[(sx, sy)] = None
    heapq.heappush(open_heap, (h(sx, sy), 0.0, sx, sy))

    while open_heap:
        _, g, cx, cy = heapq.heappop(open_heap)
        if g > g_score.get((cx, cy), 1e18):
            continue
        if (cx, cy) == (gx, gy):
            return _reconstruct(came_from, (gx, gy))
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < FC and 0 <= ny < FR):
                continue
            if not passable(nx, ny):
                continue
            ng = g + math.hypot(dx, dy) * cell_cost(nx, ny)
            if ng < g_score.get((nx, ny), 1e18):
                g_score[(nx, ny)] = ng
                came_from[(nx, ny)] = (cx, cy)
                heapq.heappush(open_heap, (ng + h(nx, ny), ng, nx, ny))

    return None


def _reconstruct(
    came_from: dict, end: tuple[int, int]
) -> list[tuple[int, int]]:
    path, node = [], end
    while node is not None:
        path.append(node)
        node = came_from[node]
    return list(reversed(path))
