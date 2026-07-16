"""Multi-agent coordination layer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from application.simulator import Simulator
from domain.types import UNKNOWN

from multi_agent.frontier_allocator import (
    FrontierAllocator,
)

from multi_agent.belief_sharing import (
    BeliefSharing,
)

from multi_agent.collision_avoidance import (
    CollisionAvoidance,
)


@dataclass
class CoordinatorState:

    tick: int = 0

    mission_complete: bool = False

    winning_drone_id: Optional[str] = None


class MultiAgentCoordinator:
    """
    Coordinates multiple independent
    single-drone simulators.
    """

    def __init__(
        self,
        simulators: list[Simulator],
        min_global_coverage: float = 0.0,
    ) -> None:

        if len(simulators) == 0:
            raise ValueError(
                "Coordinator requires at least "
                "one simulator."
            )

        self.simulators = simulators

        # Fraction (0.0-1.0) of the ENTIRE arena (union of every drone's
        # detected map) that must be scanned before certification is
        # allowed to finalize the mission. 0.0 = old behaviour.
        self.min_global_coverage = min_global_coverage

        self.state = CoordinatorState()

        self.frontier_allocator = (
            FrontierAllocator()
        )

        self.belief_sharing = (
            BeliefSharing()
        )

        self.collision_avoidance = (
            CollisionAvoidance()
        )

    # ---------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------

    def tick(self) -> bool:

        self.state.tick += 1

        # -----------------------------------------------------
        # Phase 1 - observations
        # -----------------------------------------------------

        for sim in self.simulators:

            sim.observe()

        # -----------------------------------------------------
        # Phase 2 - belief sharing
        # -----------------------------------------------------

        self.belief_sharing.synchronize(
            self.simulators,
        )

        # -----------------------------------------------------
        # Phase 3 - local planning
        # -----------------------------------------------------

        for sim in self.simulators:

            sim.mission.tick += 1

            sim.plan_corridor()

        # -----------------------------------------------------
        # Phase 4 - certification (gated on global coverage)
        # -----------------------------------------------------

        global_ratio = self.global_explored_ratio()

        for sim in self.simulators:

            certified = sim.certify()

            if certified and global_ratio >= self.min_global_coverage:

                self.state.mission_complete = True

                self.state.winning_drone_id = (
                    sim.drone_id
                )

                return True

        # -----------------------------------------------------
        # Phase 5 - frontier allocation
        # -----------------------------------------------------

        assignments = (
            self.frontier_allocator.assign(
                self.simulators,
            )
        )

        # -----------------------------------------------------
        # Phase 6 - collision filtering
        # -----------------------------------------------------

        assignments = (
            self.collision_avoidance
            .filter_assignments(
                assignments,
            )
        )

        # -----------------------------------------------------
        # Phase 7 - movement
        # -----------------------------------------------------

        for sim in self.simulators:

            nxt = assignments.get(
                sim.drone_id,
            )

            if nxt is None:
                continue

            sim.move(nxt)

        return False

    # ---------------------------------------------------------
    # Convenience methods
    # ---------------------------------------------------------

    def global_explored_ratio(self) -> float:
        """Fraction of the arena that's non-UNKNOWN in at least one
        drone's detected map (union across all drones)."""
        union = None
        for sim in self.simulators:
            mask = sim.world.detected != UNKNOWN
            union = mask if union is None else (union | mask)
        if union is None:
            return 0.0
        return float(union.mean())

    def certified(self) -> bool:

        return self.state.mission_complete

    def winner(self) -> Optional[str]:

        return self.state.winning_drone_id

    def drone_positions(
        self,
    ) -> dict[str, tuple[int, int]]:

        positions = {}

        for sim in self.simulators:

            positions[sim.drone_id] = (
                sim.drone.block
            )

        return positions