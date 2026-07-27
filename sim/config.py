from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MissionSimConfig:
    """IARC field-aligned mission sim (meters)."""

    field_x_m: float = 91.44  # 300 ft — long axis (start → goal)
    field_y_m: float = 24.38  # 80 ft — short axis (start/goal on parallel sides)
    resolution_m: float = 0.2
    clearance_m: float = 0.3
    edge_margin_m: float = 0.5
    # Flight volume (meters above ground, z=0)
    ground_z_m: float = 0.0
    default_altitude_m: float = 1.5
    min_altitude_m: float = 0.4
    max_altitude_m: float = 3.0
    # R_soft: RL shaping in sim (spread out). R_hard: safety floor (sim metrics + ESP32).
    min_separation_soft_m: float = 4.0
    min_separation_hard_m: float = 1.5

    @property
    def cols(self) -> int:
        return int(self.field_x_m / self.resolution_m)

    @property
    def rows(self) -> int:
        return int(self.field_y_m / self.resolution_m)
