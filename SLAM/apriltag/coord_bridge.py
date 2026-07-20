from __future__ import annotations


def world_to_fine(
    world_x: float,
    world_y: float,
    fine_cols: int,
    fine_rows: int,
    field_x: float,
    field_y: float,
) -> tuple[int, int]:
    fx = int(world_x / field_x * fine_cols)
    fy = int(world_y / field_y * fine_rows)
    fx = max(0, min(fine_cols - 1, fx))
    fy = max(0, min(fine_rows - 1, fy))
    return fx, fy
