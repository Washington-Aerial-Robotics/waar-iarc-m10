import numpy as np
from dataclasses import dataclass, field


@dataclass
class GroundTruth:
    fine_cols: int
    fine_rows: int
    hazard_positions: list

    _grid: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        self._grid = np.zeros((self.fine_cols, self.fine_rows), dtype=bool)
        for fx, fy in self.hazard_positions:
            if 0 <= fx < self.fine_cols and 0 <= fy < self.fine_rows:
                self._grid[fx, fy] = True

    def is_hazard(self, fx: int, fy: int) -> bool:
        return bool(self._grid[fx, fy])

    def get_hazard_positions(self) -> list:
        return list(self.hazard_positions)

    @classmethod
    def from_random(cls, fine_cols: int, fine_rows: int,
                    n_hazards: int, seed: int = 42) -> "GroundTruth":
        rng = np.random.default_rng(seed)
        xs = rng.integers(2, fine_cols - 2, size=n_hazards)
        ys = rng.integers(2, fine_rows - 2, size=n_hazards)
        positions = list(zip(xs.tolist(), ys.tolist()))
        return cls(fine_cols=fine_cols, fine_rows=fine_rows,
                   hazard_positions=positions)
