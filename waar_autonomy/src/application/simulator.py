"""Application-layer simulation loop.

Wires UC1→UC2→UC3→UC4 each tick and tracks mission state.
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
    Constructor observes the drone's starting block so map3 is never blank.
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

        # Observe starting block immediately (mirrors original World.__init__)
        observe_block(world, gt, *drone.block, inflation_radius)

    def tick(self) -> bool:
        """
        Execute one simulation step.

        Returns True when the corridor is certified (mission complete).
        """
        self.mission.tick += 1
        bx, by = self.drone.block

        # UC1 — observe current block
        observe_block(self.world, self.gt, bx, by, self.inflation_radius)

        # UC2 — recompute corridor candidate
        path = compute_best_corridor(self.world, self.unknown_cost)
        if path:
            self.mission.corridor = path

        # UC3 — certify?
        if path:
            result = evaluate_corridor(
                path, self.world,
                self.min_clearance_cells,
                self.min_coverage_ratio,
            )
            if result.certified:
                self.mission.certified      = True
                self.mission.cert_tick      = self.mission.tick
                self.mission.cert_clearance = result.clearance
                return True

        # UC4 — pick next frontier block
        nxt = best_frontier(
            self.world, self.drone, self.mission.corridor,
            self.unknown_cost, self.w_cert,
        )
        if nxt is None:
            return False

        self.drone.move_to(*nxt)
        return False


    # -----------------------------------------------------------------------
    # Step-by-step API used by the multi-agent coordinator
    # -----------------------------------------------------------------------

    def observe(self) -> None:
        """Reveal the drone's current block without moving it."""
        bx, by = self.drone.block
        observe_block(self.world, self.gt, bx, by, self.inflation_radius)

    def plan_corridor(self) -> Optional[list[tuple[int, int]]]:
        """Recompute and store the current best corridor candidate."""
        path = compute_best_corridor(self.world, self.unknown_cost)
        if path:
            self.mission.corridor = path
        return path

    def certify(self) -> bool:
        """Check whether the currently planned corridor is certified."""
        if not self.mission.corridor:
            return False

        result = evaluate_corridor(
            self.mission.corridor,
            self.world,
            self.min_clearance_cells,
            self.min_coverage_ratio,
        )

        if result.certified:
            self.mission.certified = True
            self.mission.cert_tick = self.mission.tick
            self.mission.cert_clearance = result.clearance
            return True

        return False

    def choose_frontier(
        self,
        reserved_frontiers: Optional[set[tuple[int, int]]] = None,
    ) -> Optional[tuple[int, int]]:
        """Choose the next frontier while avoiding already reserved targets."""
        return best_frontier(
            self.world,
            self.drone,
            self.mission.corridor,
            self.unknown_cost,
            self.w_cert,
            reserved_frontiers=reserved_frontiers,
        )

    def move(self, block: tuple[int, int]) -> None:
        """Move the drone to a selected block."""
        self.drone.move_to(*block)

    # -----------------------------------------------------------------------
    # Convenience metrics for display (delegates to UC3 helpers)
    # -----------------------------------------------------------------------

    def current_clearance(self) -> float:
        if not self.mission.corridor:
            return 0.0
        return corridor_clearance(self.mission.corridor, self.world)

    def current_coverage(self) -> float:
        if not self.mission.corridor:
            return 0.0
        return corridor_coverage(self.mission.corridor, self.world)
