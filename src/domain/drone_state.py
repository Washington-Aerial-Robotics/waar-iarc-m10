from __future__ import annotations
from domain.types import block_center_fine

class DroneState:
    """Position of the single drone at block and fine-grid resolution."""
    def __init__(self, block_cols: int = 20, block_rows: int = 15,
                 block: tuple[int, int] | None = None) -> None:
        self.block: tuple[int, int] = block if block is not None else (0, block_rows - 2)
        self.fine:  tuple[int, int] = block_center_fine(*self.block)

    def move_to(self, bx: int, by: int) -> None:
        self.block = (bx, by)
        self.fine  = block_center_fine(bx, by)
