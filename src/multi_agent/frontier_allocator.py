"""Frontier assignment utilities."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from application.simulator import Simulator


class FrontierAllocator:
    """
    Greedy frontier allocator.

    Prevents multiple drones from selecting
    identical frontier targets.
    """

    def assign(
        self,
        simulators: list["Simulator"],
    ) -> dict[str, tuple[int, int]]:

        assignments = {}

        reserved_frontiers = set()

        for sim in simulators:

            nxt = sim.choose_frontier(
                reserved_frontiers,
            )

            if nxt is None:
                continue

            reserved_frontiers.add(nxt)

            assignments[
                sim.drone_id
            ] = nxt

        return assignments