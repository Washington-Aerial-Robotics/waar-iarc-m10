"""Multi-agent coordination layer."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from application.simulator import Simulator

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
    ) -> None:

        if len(simulators) == 0:
            raise ValueError(
                "Coordinator requires at least "
                "one simulator."
            )

        self.simulators = simulators

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
        # Phase 1 — observations
        # -----------------------------------------------------

        for sim in self.simulators:

            sim.observe()

        # -----------------------------------------------------
        # Phase 2 — belief sharing
        # -----------------------------------------------------

        self.belief_sharing.synchronize(
            self.simulators,
        )

        # -----------------------------------------------------
        # Phase 3 — local planning
        # -----------------------------------------------------

        for sim in self.simulators:

            sim.mission.tick += 1

            sim.plan_corridor()

        # -----------------------------------------------------
        # Phase 4 — certification
        # -----------------------------------------------------

        for sim in self.simulators:

            certified = sim.certify()

            if certified:

                self.state.mission_complete = True

                self.state.winning_drone_id = (
                    sim.drone_id
                )

                return True

        # -----------------------------------------------------
        # Phase 5 — frontier allocation
        # -----------------------------------------------------

        assignments = (
            self.frontier_allocator.assign(
                self.simulators,
            )
        )

        # -----------------------------------------------------
        # Phase 6 — collision filtering
        # -----------------------------------------------------

        assignments = (
            self.collision_avoidance
            .filter_assignments(
                assignments,
            )
        )

        # -----------------------------------------------------
        # Phase 7 — movement
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