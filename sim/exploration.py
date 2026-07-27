from __future__ import annotations

import math
from dataclasses import dataclass, field

from .config import MissionSimConfig
from .drone_flight import DroneFlightModel, SerpentinePatrol
from .field import HumanPathField
from .mines import Mine
from .pathfinding import astar_human_path, path_length_m
from .perception_geometry import DroneSensorModel
from .separation import (
    SeparationSnapshot,
    classify_separation_violations,
    compute_horizontal_separation,
)


@dataclass
class DroneState:
    flight: DroneFlightModel
    patrol: SerpentinePatrol
    index: int = 0
    trail: list[tuple[float, float, float]] = field(default_factory=list)

    @property
    def x(self) -> float:
        return self.flight.x

    @property
    def y(self) -> float:
        return self.flight.y

    @property
    def z(self) -> float:
        return self.flight.z

    @property
    def yaw(self) -> float:
        return self.flight.yaw


@dataclass
class ExplorationMetrics:
    ticks: int = 0
    mines_total: int = 0
    mines_discovered: int = 0
    path_found: bool = False
    path_length_m: float = 0.0
    coverage_ratio: float = 0.0
    drone_altitudes_m: tuple[float, ...] = ()
    min_pairwise_distance_m: float | None = None
    # (drone_i, drone_j, distance_m) for pairs inside R_hard / R_soft this tick
    separation_hard_violations: tuple[tuple[int, int, float], ...] = ()
    separation_soft_violations: tuple[tuple[int, int, float], ...] = ()

    def summary(self) -> str:
        pct = 100.0 * self.mines_discovered / self.mines_total if self.mines_total else 0.0
        alt = ""
        if self.drone_altitudes_m:
            alt = "  z_m=" + ",".join(f"{z:.1f}" for z in self.drone_altitudes_m)
        sep = ""
        if self.min_pairwise_distance_m is not None:
            sep = f"  sep_min={self.min_pairwise_distance_m:.1f}m"
            if self.separation_hard_violations:
                sep += f"  HARD={len(self.separation_hard_violations)}"
        return (
            f"ticks={self.ticks}  mines={self.mines_discovered}/{self.mines_total} ({pct:.0f}%)  "
            f"path={'yes' if self.path_found else 'no '}  "
            f"length={self.path_length_m:.1f}m  coverage={self.coverage_ratio*100:.0f}%{alt}{sep}"
        )


class ExplorationSim:
    """
    Drones patrol the field; mines enter the map when within sensor range.
    Human path is recomputed from discovered mines only.
    """

    def __init__(
        self,
        config: MissionSimConfig,
        truth_mines: list[Mine],
        *,
        num_drones: int = 4,
        sensor_range_m: float = 4.0,
        legacy_patrol: bool = False,
        sensor: DroneSensorModel | None = None,
    ):
        self.config = config
        self.truth_mines = list(truth_mines)
        self.legacy_patrol = legacy_patrol
        self.sensor = sensor or DroneSensorModel(
            ref_altitude_m=config.default_altitude_m,
            ref_ground_range_m=sensor_range_m,
        )
        self.discovered: dict[int, Mine] = {}
        self.field = HumanPathField(config)
        self.path: list[tuple[int, int]] | None = None
        self.metrics = ExplorationMetrics(mines_total=len(truth_mines))
        self._visited: set[tuple[int, int]] = set()

        margin = config.edge_margin_m
        self.drones: list[DroneState] = []
        for i in range(max(1, num_drones)):
            y = config.field_y_m * (i + 1) / (num_drones + 1)
            y = max(margin, min(config.field_y_m - margin, y))
            flight = DroneFlightModel(x=margin, y=y, z=config.default_altitude_m)
            flight.set_altitude_limits(config.min_altitude_m, config.max_altitude_m)
            flight.arm()
            patrol = SerpentinePatrol(
                config.field_x_m,
                config.field_y_m,
                margin,
                lane_y=y,
                phase_x_m=i * 6.0,
                target_z_m=config.default_altitude_m,
            )
            self.drones.append(
                DroneState(
                    flight=flight,
                    patrol=patrol,
                    index=i,
                    trail=[(margin, y, config.default_altitude_m)],
                )
            )

        self._last_replan_discovered = 0

    def discover_from_csv_row(self, mine: Mine) -> bool:
        """Replay mode: add mine when perception log reports it."""
        if mine.tag_id in self.discovered:
            return False
        self.discovered[mine.tag_id] = mine
        self.field.rebuild_from_mines(list(self.discovered.values()))
        self._replan()
        return True

    def step(self) -> ExplorationMetrics:
        self.metrics.ticks += 1
        for drone in self.drones:
            self._move_drone(drone)
            self._discover_near(drone)
            row, col = self.field.world_to_cell(drone.x, drone.y)
            self._visited.add((row, col))

        self._replan()
        total_cells = self.config.rows * self.config.cols
        self.metrics.mines_discovered = len(self.discovered)
        self.metrics.path_found = self.path is not None
        self.metrics.path_length_m = path_length_m(self.field, self.path) if self.path else 0.0
        self.metrics.coverage_ratio = len(self._visited) / total_cells if total_cells else 0.0
        self.metrics.drone_altitudes_m = tuple(d.z for d in self.drones)
        sep = self.compute_drone_separation()
        self.metrics.min_pairwise_distance_m = sep.min_pairwise_distance_m
        self.metrics.separation_hard_violations = tuple(
            (p.i, p.j, p.distance_m) for p in sep.hard_violations
        )
        self.metrics.separation_soft_violations = tuple(
            (p.i, p.j, p.distance_m) for p in sep.soft_violations
        )
        return self.metrics

    def compute_drone_separation(self) -> SeparationSnapshot:
        """Pairwise horizontal distances (x-y); no position clamping — use for RL reward."""
        positions = [(d.x, d.y) for d in self.drones]
        snap = compute_horizontal_separation(positions)
        return classify_separation_violations(
            snap,
            r_soft_m=self.config.min_separation_soft_m,
            r_hard_m=self.config.min_separation_hard_m,
        )

    def _move_drone(self, drone: DroneState) -> None:
        cfg = self.config
        if self.legacy_patrol:
            self._move_drone_legacy(drone)
            return

        sticks = drone.patrol.sticks_for_pose(
            drone.flight.x,
            drone.flight.y,
            drone.flight.z,
            drone.flight.yaw,
            drone.flight.hover_throttle,
        )
        drone.flight.set_sticks(*sticks)
        drone.flight.controls_step(
            field_x_m=cfg.field_x_m,
            field_y_m=cfg.field_y_m,
            margin_m=cfg.edge_margin_m,
        )
        drone.trail.append((drone.flight.x, drone.flight.y, drone.flight.z))
        if len(drone.trail) > 800:
            drone.trail = drone.trail[-800:]

    def _move_drone_legacy(self, drone: DroneState) -> None:
        """Old grid-step patrol (not representative of real flight)."""
        cfg = self.config
        margin = cfg.edge_margin_m
        step_m = cfg.resolution_m
        if not hasattr(self, "_serpentine_dir"):
            self._serpentine_dir = 1
        drone.flight.x += step_m
        drone.flight.y += self._serpentine_dir * step_m * 0.35

        if drone.flight.y >= cfg.field_y_m - margin:
            drone.flight.y = cfg.field_y_m - margin
            self._serpentine_dir = -1
        elif drone.flight.y <= margin:
            drone.flight.y = margin
            self._serpentine_dir = 1

        if drone.flight.x > cfg.field_x_m - margin:
            drone.flight.x = margin

        drone.trail.append((drone.flight.x, drone.flight.y, drone.flight.z))
        if len(drone.trail) > 800:
            drone.trail = drone.trail[-800:]

    def _discover_near(self, drone: DroneState) -> None:
        if self.legacy_patrol:
            radius = self.sensor.ref_ground_range_m
            for mine in self.truth_mines:
                if mine.tag_id in self.discovered:
                    continue
                dist = math.hypot(mine.world_x - drone.x, mine.world_y - drone.y)
                if dist <= radius:
                    self.discovered[mine.tag_id] = mine
        else:
            for mine in self.truth_mines:
                if mine.tag_id in self.discovered:
                    continue
                if self.sensor.can_detect_ground_point(
                    drone_x=drone.x,
                    drone_y=drone.y,
                    drone_z=drone.z,
                    point_x=mine.world_x,
                    point_y=mine.world_y,
                ):
                    self.discovered[mine.tag_id] = mine
        if self.discovered:
            self.field.rebuild_from_mines(list(self.discovered.values()))

    def _replan(self) -> None:
        n = len(self.discovered)
        if n == 0:
            self.path = None
            return
        if n == self._last_replan_discovered:
            return
        self._last_replan_discovered = n
        self.path = astar_human_path(self.field)
