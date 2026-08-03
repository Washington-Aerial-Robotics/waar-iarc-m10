from __future__ import annotations

K = 4  # fine cells per block side

# Map 3 cell states
UNKNOWN  = 0
FREE     = 1
HAZARD   = 2
INFLATED = 3


def block_to_fine_origin(bx: int, by: int) -> tuple[int, int]:
    return bx * K, by * K


def fine_cells_of_block(bx: int, by: int) -> list[tuple[int, int]]:
    ox, oy = block_to_fine_origin(bx, by)
    return [(ox + dx, oy + dy) for dx in range(K) for dy in range(K)]


def block_center_fine(bx: int, by: int) -> tuple[int, int]:
    ox, oy = block_to_fine_origin(bx, by)
    return ox + K // 2, oy + K // 2
