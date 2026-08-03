from __future__ import annotations

from domain.types import block_center_fine


class DroneState:
    """Position of a drone at block and fine-grid resolution."""

    def __init__(
        self,
        block_cols: int | None = None,
        block_rows: int | None = None,
        block: tuple[int, int] | None = None,
    ) -> None:
        if block is not None:
            self.block = block
        else:
            if block_cols is None or block_rows is None:
                raise ValueError(
                    "Provide either block=(bx, by), or block_cols and block_rows."
                )
            self.block = (0, block_rows - 2)

        self.fine: tuple[int, int] = block_center_fine(*self.block)

    def move_to(self, bx: int, by: int) -> None:
        self.block = (bx, by)
        self.fine = block_center_fine(bx, by)
