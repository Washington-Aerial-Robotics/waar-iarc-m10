"""UC4 - Score Frontiers (Corridor-Aware).

Ranks unvisited frontier blocks by a score that combines:
  - proximity to the goal line (reverse-Dijkstra cost, multi-source from goal_line)
  - travel cost from drone position
  - certification gain (UNKNOWN cells overlapping current corridor)
"""
from __future__ import annotations

import heapq
import math
from typing import Optional

import numpy as np

from domain.types import UNKNOWN, HAZARD, INFLATED, fine_cells_of_block, block_center_fine
from domain.world_model import WorldModel
from domain.drone_state import DroneState


def best_frontier(
    world: WorldModel,
    drone: DroneState,
    corridor: Optional[list[tuple[int, int]]],
    unknown_cost: float,
    w_cert: float,
) -> Optional[tuple[int, int]]:
    """Return the highest-scoring frontier block, or None if no frontiers exist."""
    fronts = _frontier_blocks(world)
    if not fronts:
        return None

    cost_to_goal = _reverse_dijkstra(world, unknown_cost)
    return max(
        fronts,
        key=lambda b: _score_block(b[0], b[1], world, drone, corridor, cost_to_goal, w_cert),
    )


def _frontier_blocks(world: WorldModel) -> list[tuple[int, int]]:
    """Unvisited blocks 4-connected to at least one visited block."""
    BC, BR = world.block_cols, world.block_rows
    result = []
    for bx in range(BC):
        for by in range(BR):
            if world.visited[bx, by]:
                continue
            for dbx, dby in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = bx + dbx, by + dby
                if 0 <= nx < BC and 0 <= ny < BR and world.visited[nx, ny]:
                    result.append((bx, by))
                    break
    return result


def _reverse_dijkstra(world: WorldModel, unknown_cost: float) -> np.ndarray:
    """Dijkstra backwards from every cell on goal_line (multi-source);
    impassable cells get np.inf."""
    FC, FR = world.fine_cols, world.fine_rows
    detected = world.detected

    cost_to_goal = np.full((FC, FR), np.inf, dtype=float)
    heap: list = []

    for gx, gy in world.goal_line:
        if detected[gx, gy] in (HAZARD, INFLATED):
            continue
        cost_to_goal[gx, gy] = 0.0
        heapq.heappush(heap, (0.0, gx, gy))

    while heap:
        g, cx, cy = heapq.heappop(heap)
        if g > cost_to_goal[cx, cy]:
            continue
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < FC and 0 <= ny < FR):
                continue
            if detected[nx, ny] in (HAZARD, INFLATED):
                continue
            ec = unknown_cost if detected[nx, ny] == UNKNOWN else 1.0
            ng = g + math.hypot(dx, dy) * ec
            if ng < cost_to_goal[nx, ny]:
                cost_to_goal[nx, ny] = ng
                heapq.heappush(heap, (ng, nx, ny))

    return cost_to_goal


def _certification_gain(
    bx: int,
    by: int,
    world: WorldModel,
    corridor: Optional[list[tuple[int, int]]],
) -> float:
    """Count UNKNOWN fine cells in block (bx, by) that lie on the corridor."""
    if corridor is None:
        return 0.0
    corridor_set = set(corridor)
    return float(
        sum(
            1
            for fx, fy in fine_cells_of_block(bx, by)
            if world.detected[fx, fy] == UNKNOWN and (fx, fy) in corridor_set
        )
    )


def _score_block(
    bx: int,
    by: int,
    world: WorldModel,
    drone: DroneState,
    corridor: Optional[list[tuple[int, int]]],
    cost_to_goal: np.ndarray,
    w_cert: float,
) -> float:
    fx, fy   = block_center_fine(bx, by)
    dfx, dfy = drone.fine

    goal_cost = cost_to_goal[fx, fy]
    if math.isinf(goal_cost):
        return -math.inf

    travel_cost = math.hypot(fx - dfx, fy - dfy)
    cert_gain   = _certification_gain(bx, by, world, corridor)
    return -(travel_cost + goal_cost) + w_cert * cert_gain
