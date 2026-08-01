from __future__ import annotations

from dataclasses import dataclass

from domain.types import K


@dataclass
class Config:
    block_cols: int   = 20
    block_rows: int   = 15
    n_hazards:  int   = 25

    inflation_radius:    float = 3.5
    unknown_cost:        float = 4.0
    min_clearance_cells: float = 2.0
    min_coverage_ratio:  float = 1.0
    w_cert:              float = 4.0

    max_ticks:  int   = 600
    tick_delay: float = 0.06

    @property
    def fine_cols(self) -> int:
        return self.block_cols * K

    @property
    def fine_rows(self) -> int:
        return self.block_rows * K
