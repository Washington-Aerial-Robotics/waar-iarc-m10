"""
Inter-drone horizontal separation (sim).

R_soft: RL shaping — encourage spreading (reward penalty only in sim).
R_hard: physical safety floor — mirrored on ESP32 firmware; never violate in flight.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class DronePairDistance:
    i: int
    j: int
    distance_m: float


@dataclass
class SeparationSnapshot:
    """Pairwise horizontal (x-y) separation for one simulation tick."""

    pairs: list[DronePairDistance] = field(default_factory=list)
    min_pairwise_distance_m: float | None = None
    hard_violations: list[DronePairDistance] = field(default_factory=list)
    soft_violations: list[DronePairDistance] = field(default_factory=list)


def compute_horizontal_separation(
    positions_xy: list[tuple[float, float]],
) -> SeparationSnapshot:
    """All unordered pairs; ignores altitude (shallow field, horizontal collision risk)."""
    snap = SeparationSnapshot()
    n = len(positions_xy)
    if n < 2:
        return snap

    min_d = math.inf
    for i in range(n):
        xi, yi = positions_xy[i]
        for j in range(i + 1, n):
            xj, yj = positions_xy[j]
            d = math.hypot(xj - xi, yj - yi)
            pair = DronePairDistance(i, j, d)
            snap.pairs.append(pair)
            min_d = min(min_d, d)

    snap.min_pairwise_distance_m = min_d if min_d != math.inf else None
    return snap


def classify_separation_violations(
    snap: SeparationSnapshot,
    *,
    r_soft_m: float,
    r_hard_m: float,
) -> SeparationSnapshot:
    """Fill hard_violations / soft_violations on snap (d < thresholds)."""
    snap.hard_violations = [p for p in snap.pairs if p.distance_m < r_hard_m]
    # Soft band: hard pairs also count as soft crowding
    snap.soft_violations = [p for p in snap.pairs if p.distance_m < r_soft_m]
    return snap


def separation_reward_penalty(
    snap: SeparationSnapshot,
    *,
    r_soft_m: float,
    r_hard_m: float,
    hard_pair_penalty: float = 10.0,
) -> float:
    """
    Summed penalty for RL: subtract epsilon * this from reward.

    Soft: for each pair with d < R_soft, add (R_soft - d) / R_soft.
    Hard: add fixed large penalty per pair with d < R_hard.
    """
    if not snap.pairs:
        return 0.0

    if not snap.soft_violations and not snap.hard_violations:
        classify_separation_violations(snap, r_soft_m=r_soft_m, r_hard_m=r_hard_m)

    penalty = 0.0
    for pair in snap.soft_violations:
        penalty += (r_soft_m - pair.distance_m) / r_soft_m
    penalty += hard_pair_penalty * len(snap.hard_violations)
    return penalty
