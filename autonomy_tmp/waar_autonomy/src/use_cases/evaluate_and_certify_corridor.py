"""UC3 — Evaluate and Certify Corridor.

Computes clearance (distance from nearest hazard) and coverage (fraction of
path cells that are FREE) and certifies if both thresholds are met.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from domain.types import FREE, HAZARD
from domain.world_model import WorldModel
from use_cases.types import CertificationResult


def evaluate_corridor(
    corridor: Optional[list[tuple[int, int]]],
    world: WorldModel,
    min_clearance_cells: float,
    min_coverage_ratio: float,
) -> CertificationResult:
    if not corridor:
        return CertificationResult(certified=False, clearance=0.0, coverage=0.0)

    clr = corridor_clearance(corridor, world)
    cov = corridor_coverage(corridor, world)
    return CertificationResult(
        certified=clr >= min_clearance_cells and cov >= min_coverage_ratio,
        clearance=clr,
        coverage=cov,
    )


def corridor_clearance(
    path: list[tuple[int, int]], world: WorldModel
) -> float:
    hxs, hys = np.where(world.detected == HAZARD)
    if len(hxs) == 0:
        return float("inf")
    min_d = float("inf")
    for px, py in path:
        ds = np.hypot(hxs - px, hys - py)
        min_d = min(min_d, float(ds.min()))
    return min_d


def corridor_coverage(
    path: list[tuple[int, int]], world: WorldModel
) -> float:
    if not path:
        return 0.0
    free = sum(1 for x, y in path if world.detected[x, y] == FREE)
    return free / len(path)
