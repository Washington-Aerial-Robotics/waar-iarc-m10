"""Application-layer simulation loop.

Wires UC1->UC2->UC3->UC4 each tick and tracks mission state.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from domain.world_model import WorldModel
from domain.drone_state import DroneState
from ports.ground_truth_port import GroundTruthPort
from use_cases.update_world_model import observe_block
from use_cases.compute_best_corridor import compute_best_corridor
from use_cases.evaluate_and_certify_corridor import (
    evaluate_corridor, corridor_clearance, corridor_coverage,
)
from use_cases.score_frontiers_corridor_aware import best_frontier


@dataclass
class MissionState:
    tick:           int   = 0
    corridor:       Optional[list[tuple[int, int]]] = field(default=None)
    certified:      bool  = False
    cert_tick:      Optional[int]   = None
    cert_clearance: Optional[float] = None


class Simulator:
    """
    Single-drone simulator.

    Owns WorldModel and DroneState; calls use cases each tick.
    Constructor observes the drone starting block so map3 is never blank.
    """

    def __init__(
        self,
        world:                WorldModel,
        drone:                DroneState,
        gt:                   GroundTruthPort,
        inflation_radius:     float,
        unknown_cost:         float,
        min_clearance_cells:  float,
        min_coverage_ratio:   float,
        w_cert:               float,
        drone_id:             str = "D0",
    ) -> None:
        self.drone_id            = drone_id
        self.world               = world
        self.drone               = drone
        self.gt                  = gt
        self.inflation_radius    = inflation_radius
        self.unknown_cost        = unknown_cost
        self.min_clearance_cells = min_clearance_cells
        self.min_coverage_ratio  = min_coverage_ratio
        self.w_cert              = w_cert
        self.mission             = MissionState()
        self._next_block: Optional[tuple[int, int]] = None

        # Observe starting block immediately
        observe_block(world, gt, *drone.block, inflation_radius)

    # ------------------------------------------------------------------
    # Split methods used by MultiAgentCoordinator
    # ------------------------------------------------------------------

    def observe(self) -> None:
        """UC1 - reveal cells around current block."""
        bx, by = self.drone.block
        observe_block(self.world, self.gt, bx, by, self.inflation_radius)

    def plan_corridor(self) -> None:
        """UC2 - recompute corridor candidate."""
        path = compute_best_corridor(self.world, self.unknown_cost)
        if path:
            self.mission.corridor = path

    def certify(self) -> bool:
        """UC3 - check if corridor is certified. Returns True if done."""
        if not self.mission.corridor:
            return False
        result = evaluate_corridor(
            self.mission.corridor, self.world,
            self.min_clearance_cells,
            self.min_coverage_ratio,
        )
        if result.certified:
            self.mission.certified      = True
            self.mission.cert_tick      = self.mission.tick
            self.mission.cert_clearance = result.clearance
            return True
        return False

    def select_frontier(self) -> Optional[tuple[int, int]]:
        """UC4 - score and return the best next frontier block."""
        nxt = best_frontier(
            self.world, self.drone, self.mission.corridor,
            self.unknown_cost, self.w_cert,
        )
        self._next_block = nxt
        return nxt

    def move(self, block: tuple[int, int]) -> None:
        """Move drone to the given block."""
        self.drone.move_to(*block)

    def choose_frontier(
        self,
        reserved: set[tuple[int, int]] | None = None,
    ) -> "Optional[tuple[int, int]]":
        """UC4 - like select_frontier but skips reserved blocks."""
        from use_cases.score_frontiers_corridor_aware import (
            _frontier_blocks, _reverse_dijkstra, _score_block,
        )
        reserved = reserved or set()
        fronts = [b for b in _frontier_blocks(self.world) if b not in reserved]
        if not fronts:
            return None
        cost_to_goal = _reverse_dijkstra(self.world, self.unknown_cost)
        return max(
            fronts,
            key=lambda b: _score_block(
                b[0], b[1], self.world, self.drone,
                self.mission.corridor, cost_to_goal, self.w_cert,
            ),
        )
    # ------------------------------------------------------------------
    # Original single-drone tick (kept for backward compatibility)
    # ------------------------------------------------------------------

    def tick(self) -> bool:
        """
        Execute one simulation step.
        Returns True when the corridor is certified (mission complete).
        """
        self.mission.tick += 1
        self.observe()
        self.plan_corridor()
        if self.certify():
            return True
        nxt = self.select_frontier()
        if nxt is None:
            return False
        self.move(nxt)
        return False

    # ------------------------------------------------------------------
    # Convenience metrics
    # ------------------------------------------------------------------

    def current_clearance(self) -> float:
        if not self.mission.corridor:
            return 0.0
        return corridor_clearance(self.mission.corridor, self.world)

    def current_coverage(self) -> float:
        if not self.mission.corridor:
            return 0.0
        return corridor_coverage(self.mission.corridor, self.world)

