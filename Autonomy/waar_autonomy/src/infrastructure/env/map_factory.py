from __future__ import annotations

import random

from domain.types import K
from infrastructure.env.ground_truth_map import GroundTruthMap


def build_ground_truth_map(
    block_cols: int, block_rows: int, n_hazards: int, seed: int
) -> GroundTruthMap:
    """Place *n_hazards* hazard cells at random, avoiding the boundary margin."""
    FC = block_cols * K
    FR = block_rows * K
    rng = random.Random(seed)
    gt = GroundTruthMap(FC, FR)

    placed = attempts = 0
    while placed < n_hazards and attempts < 10_000:
        fx = rng.randint(K * 2, FC - K * 2 - 1)
        fy = rng.randint(1, FR - 2)
        if not gt.data[fx, fy]:
            gt.data[fx, fy] = True
            placed += 1
        attempts += 1

    return gt
