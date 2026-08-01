from __future__ import annotations

from domain.types import block_center_fine


class DroneState:
    """Position of the single drone at block and fine-grid resolution."""

    def __init__(self, block_cols: int, block_rows: int) -> None:
        self.block: tuple[int, int] = (0, block_rows - 2)
        self.fine:  tuple[int, int] = block_center_fine(*self.block)

    def move_to(self, bx: int, by: int) -> None:
        self.block = (bx, by)
        self.fine  = block_center_fine(bx, by)
