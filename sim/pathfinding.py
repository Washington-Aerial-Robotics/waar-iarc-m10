from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .field import HumanPathField
from .types import FREE

# IARC: 1 foot = 0.3048 m
FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M

NEIGHBORS_8 = (
    (1, 0),
    (-1, 0),
    (0, 1),
    (0, -1),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
)


@dataclass
class HumanPathResult:
    """Safe human corridor from start edge → far edge over discovered mines only."""

    path_cells: list[tuple[int, int]] | None
    width_m: float  # narrowest corridor width along path (2 * min clearance to blocked)
    length_m: float  # centerline length
    min_clearance_m: float  # min distance from path to blocked cells
    found: bool = False

    @property
    def width_ft(self) -> float:
        return self.width_m * M_TO_FT

    @property
    def length_ft(self) -> float:
        return self.length_m * M_TO_FT

    def summary(self) -> str:
        if not self.found or self.path_cells is None:
            return "NO SAFE PATH FOUND"
        return (
            f"SAFE PATH  W={self.width_ft:.2f} ft  L={self.length_ft:.1f} ft  "
            f"({self.width_m:.3f} m × {self.length_m:.2f} m)"
        )


def clearance_field_m(field: HumanPathField) -> np.ndarray:
    """
    Euclidean distance (m) from each cell center to the nearest blocked cell.
    Blocked cells have clearance 0.
    """
    blocked = field.grid != FREE
    if not np.any(blocked):
        # Entire field free — clearance limited by distance to long edges (y).
        rows, cols = field.config.rows, field.config.cols
        res = field.config.resolution_m
        yy = (np.arange(rows) + 0.5) * res
        dist_edge = np.minimum(yy, field.config.field_y_m - yy)[:, None]
        return np.broadcast_to(dist_edge, (rows, cols)).astype(np.float64).copy()

    try:
        from scipy.ndimage import distance_transform_edt

        dist_cells = distance_transform_edt(~blocked)
    except ImportError:
        dist_cells = _edt_fallback(~blocked)

    return dist_cells.astype(np.float64) * field.config.resolution_m


def _edt_fallback(free_mask: np.ndarray) -> np.ndarray:
    """Brute-force EDT when scipy is unavailable (OK for occasional replan)."""
    rows, cols = free_mask.shape
    blocked_pts = np.argwhere(~free_mask)
    if blocked_pts.size == 0:
        return np.full((rows, cols), 1e9, dtype=np.float64)
    out = np.zeros((rows, cols), dtype=np.float64)
    for r in range(rows):
        for c in range(cols):
            if not free_mask[r, c]:
                out[r, c] = 0.0
                continue
            d2 = (blocked_pts[:, 0] - r) ** 2 + (blocked_pts[:, 1] - c) ** 2
            out[r, c] = math.sqrt(float(d2.min()))
    return out


def _reachable_with_min_clearance(
    field: HumanPathField,
    clearance: np.ndarray,
    min_c: float,
) -> bool:
    start = [
        (r, c)
        for r, c in field.start_cells()
        if field.passable(r, c) and clearance[r, c] + 1e-9 >= min_c
    ]
    goals = {
        (r, c)
        for r, c in field.goal_cells()
        if field.passable(r, c) and clearance[r, c] + 1e-9 >= min_c
    }
    if not start or not goals:
        return False
    stack = list(start)
    seen = set(start)
    rows, cols = field.config.rows, field.config.cols
    while stack:
        r, c = stack.pop()
        if (r, c) in goals:
            return True
        for dr, dc in NEIGHBORS_8:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if (nr, nc) in seen:
                continue
            if not field.passable(nr, nc):
                continue
            if clearance[nr, nc] + 1e-9 < min_c:
                continue
            seen.add((nr, nc))
            stack.append((nr, nc))
    return False


def _shortest_path_with_min_clearance(
    field: HumanPathField,
    clearance: np.ndarray,
    min_c: float,
) -> Optional[list[tuple[int, int]]]:
    """A* among cells with clearance >= min_c; minimize centerline length."""
    start_cells = [
        (r, c)
        for r, c in field.start_cells()
        if field.passable(r, c) and clearance[r, c] + 1e-9 >= min_c
    ]
    goal_set = {
        (r, c)
        for r, c in field.goal_cells()
        if field.passable(r, c) and clearance[r, c] + 1e-9 >= min_c
    }
    if not start_cells or not goal_set:
        return None

    goal_cols = [c for _, c in goal_set]
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

    rows, cols = field.config.rows, field.config.cols
    while open_heap:
        _, g, row, col = heapq.heappop(open_heap)
        if g > g_score.get((row, col), 1e18):
            continue
        if (row, col) in goal_set:
            return _reconstruct(came_from, (row, col))

        for dr, dc in NEIGHBORS_8:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if not field.passable(nr, nc):
                continue
            if clearance[nr, nc] + 1e-9 < min_c:
                continue
            ng = g + math.hypot(dr, dc)
            if ng < g_score.get((nr, nc), 1e18):
                g_score[(nr, nc)] = ng
                came_from[(nr, nc)] = (row, col)
                heapq.heappush(open_heap, (ng + h(nr, nc), ng, nr, nc))
    return None


def plan_human_path(field: HumanPathField) -> HumanPathResult:
    """
    Widest safe corridor (max-min clearance) from start edge to far edge.

    Blocked = discovered-mine inflation discs (1 ft radius by config).
    W = 2 * min_clearance along the centerline (corridor width).
    L = centerline length.
    """
    clearance = clearance_field_m(field)
    free = field.grid == FREE
    if not np.any(free):
        return HumanPathResult(None, 0.0, 0.0, 0.0, found=False)

    # Binary-search maximum feasible min-clearance
    lo, hi = 0.0, float(np.max(clearance))
    best_c = 0.0
    if not _reachable_with_min_clearance(field, clearance, 0.0):
        return HumanPathResult(None, 0.0, 0.0, 0.0, found=False)

    for _ in range(28):
        mid = 0.5 * (lo + hi)
        if _reachable_with_min_clearance(field, clearance, mid):
            best_c = mid
            lo = mid
        else:
            hi = mid

    path = _shortest_path_with_min_clearance(field, clearance, best_c)
    if path is None:
        # Tiny numeric miss — retry with slightly lower threshold
        path = _shortest_path_with_min_clearance(field, clearance, max(0.0, best_c * 0.999))
    if path is None:
        return HumanPathResult(None, 0.0, 0.0, 0.0, found=False)

    min_c = min(float(clearance[r, c]) for r, c in path)
    length = path_length_m(field, path)
    width = 2.0 * min_c
    return HumanPathResult(
        path_cells=path,
        width_m=width,
        length_m=length,
        min_clearance_m=min_c,
        found=True,
    )


def astar_human_path(field: HumanPathField) -> Optional[list[tuple[int, int]]]:
    """Backward-compatible: return max-width path cells, or None if blocked."""
    result = plan_human_path(field)
    return result.path_cells if result.found else None


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
