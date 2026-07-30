from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MissionSimConfig:
    """IARC field-aligned mission sim (meters)."""

    field_x_m: float = 91.44  # 300 ft — long axis (start → goal)
    field_y_m: float = 24.38  # 80 ft — short axis (start/goal on parallel sides)
    resolution_m: float = 0.2
    clearance_m: float = 0.3048  # IARC: exactly 1 foot inflation around discovered mines
    edge_margin_m: float = 0.5
    # Flight volume (meters above ground, z=0)
    ground_z_m: float = 0.0
    default_altitude_m: float = 1.5
    min_altitude_m: float = 0.4
    max_altitude_m: float = 3.0
    # R_soft: RL shaping in sim (spread out). R_hard: safety floor (sim metrics + ESP32).
    min_separation_soft_m: float = 4.0
    min_separation_hard_m: float = 1.5
    # Lane coverage cruise speed (m/s along X). Tunable placeholder; retune from footage later.
    search_speed_m_s: float = 2.0
    # Return leg Y offset from lane center (m); None = half lane width per drone.
    return_offset_m: float | None = None
    # Coverage legs before landing (1 outbound + 1 offset return = 2 with current footprint).
    num_passes: int = 2
    # Mission clock: physics tick is 0.25 s; IARC survey window is 7 minutes.
    control_dt_s: float = 0.25
    survey_limit_s: float = 7.0 * 60.0  # survey_time_limit_s alias
    survey_time_limit_s: float | None = None  # if set, overrides survey_limit_s

    def __post_init__(self) -> None:
        if self.survey_time_limit_s is not None:
            self.survey_limit_s = float(self.survey_time_limit_s)

    @property
    def cols(self) -> int:
        return int(self.field_x_m / self.resolution_m)

    @property
    def rows(self) -> int:
        return int(self.field_y_m / self.resolution_m)

    @property
    def survey_limit_ticks(self) -> int:
        return int(round(self.survey_limit_s / self.control_dt_s))
