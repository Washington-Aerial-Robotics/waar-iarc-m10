"""UC1 — Update World Model.

Reveals fine cells of a visited block from ground truth and propagates
the hazard inflation buffer into map3.
"""
from __future__ import annotations

import math

import numpy as np

from domain.types import K, UNKNOWN, FREE, HAZARD, INFLATED, fine_cells_of_block
from domain.world_model import WorldModel
from ports.ground_truth_port import GroundTruthPort


def observe_block(
    world: WorldModel,
    gt: GroundTruthPort,
    bx: int,
    by: int,
    inflation_radius: float,
) -> None:
    """Mark block (bx, by) visited, reveal its K×K fine cells, recompute inflation."""
    world.visited[bx, by] = True
    new_hazard = False

    for fx, fy in fine_cells_of_block(bx, by):
        if gt.is_hazard(fx, fy):
            world.detected[fx, fy] = HAZARD
            new_hazard = True
        elif world.detected[fx, fy] == UNKNOWN:
            world.detected[fx, fy] = FREE

    if new_hazard:
        _recompute_inflation(world, inflation_radius)


def _recompute_inflation(world: WorldModel, inflation_radius: float) -> None:
    FC, FR = world.fine_cols, world.fine_rows
    r = int(math.ceil(inflation_radius))

    # Reset existing INFLATED cells back to FREE or UNKNOWN
    for fx in range(FC):
        for fy in range(FR):
            if world.detected[fx, fy] == INFLATED:
                bx, by = fx // K, fy // K
                world.detected[fx, fy] = FREE if world.visited[bx, by] else UNKNOWN

    # Re-apply inflation around every confirmed HAZARD
    hxs, hys = np.where(world.detected == HAZARD)
    for hx, hy in zip(hxs.tolist(), hys.tolist()):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                nx, ny = hx + dx, hy + dy
                if not (0 <= nx < FC and 0 <= ny < FR):
                    continue
                if math.hypot(dx, dy) > inflation_radius:
                    continue
                if world.detected[nx, ny] in (FREE, UNKNOWN):
                    world.detected[nx, ny] = INFLATED
