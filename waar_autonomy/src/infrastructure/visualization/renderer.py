"""Rendering helpers for the two-panel simulation display.

build_truth_image  — Map 1 (ground truth, revealed progressively)
build_detected_image — Map 3 (drone's belief: UNKNOWN/FREE/HAZARD/INFLATED)

Both return (fine_rows, fine_cols, 3) float arrays ready for imshow.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from domain.types import K, UNKNOWN, FREE, HAZARD, INFLATED, fine_cells_of_block
from domain.world_model import WorldModel
from infrastructure.env.ground_truth_map import GroundTruthMap

_COLOURS: dict[int, np.ndarray] = {
    UNKNOWN:  np.array([0.12, 0.12, 0.18]),
    FREE:     np.array([0.72, 0.76, 0.78]),
    HAZARD:   np.array([0.90, 0.15, 0.15]),
    INFLATED: np.array([0.95, 0.55, 0.15]),
}


def build_truth_image(
    world:    WorldModel,
    gt:       GroundTruthMap,
    corridor: Optional[list[tuple[int, int]]] = None,
    drone_fine: Optional[tuple[int, int]]     = None,
) -> np.ndarray:
    FC, FR = world.fine_cols, world.fine_rows
    img = np.full((FR, FC, 3), [0.18, 0.18, 0.18], dtype=float)

    for bx in range(world.block_cols):
        for by in range(world.block_rows):
            if world.visited[bx, by]:
                for fx, fy in fine_cells_of_block(bx, by):
                    if not gt.data[fx, fy]:
                        img[fy, fx] = [0.28, 0.35, 0.28]

    hxs, hys = np.where(gt.data)
    for fx, fy in zip(hxs.tolist(), hys.tolist()):
        img[fy, fx] = [0.90, 0.15, 0.15]

    _draw_grid(img, world.block_cols, world.block_rows, FC, FR)

    if corridor:
        for px, py in corridor:
            img[py, px] = [0.0, 1.0, 0.55]

    _draw_markers(img, world.start_fine, world.goal_fine, drone_fine, FC, FR)
    return img


def build_detected_image(
    world:    WorldModel,
    corridor: Optional[list[tuple[int, int]]] = None,
    drone_fine: Optional[tuple[int, int]]     = None,
) -> np.ndarray:
    FC, FR = world.fine_cols, world.fine_rows
    img = np.zeros((FR, FC, 3), dtype=float)

    for state, col in _COLOURS.items():
        mask = (world.detected == state).T
        img[mask] = col

    _draw_grid(img, world.block_cols, world.block_rows, FC, FR)

    if corridor:
        for px, py in corridor:
            img[py, px] = [0.0, 1.0, 0.55]

    _draw_markers(img, world.start_fine, world.goal_fine, drone_fine, FC, FR)
    return img


def _draw_grid(img, bc, br, FC, FR):
    for bx in range(bc + 1):
        fx = min(bx * K, FC - 1)
        img[:, fx] = img[:, fx] * 0.55 + 0.2
    for by in range(br + 1):
        fy = min(by * K, FR - 1)
        img[fy, :] = img[fy, :] * 0.55 + 0.2


def _draw_markers(img, start, goal, drone, FC, FR):
    sx, sy = start
    gx, gy = goal
    for ddx in range(-1, 2):
        for ddy in range(-1, 2):
            if 0 <= sx + ddx < FC and 0 <= sy + ddy < FR:
                img[sy + ddy, sx + ddx] = [0.2, 0.9, 0.2]
            if 0 <= gx + ddx < FC and 0 <= gy + ddy < FR:
                img[gy + ddy, gx + ddx] = [0.2, 0.5, 1.0]
    if drone:
        dfx, dfy = drone
        for ddx in range(-1, 2):
            for ddy in range(-1, 2):
                if 0 <= dfx + ddx < FC and 0 <= dfy + ddy < FR:
                    img[dfy + ddy, dfx + ddx] = [1.0, 1.0, 0.0]
