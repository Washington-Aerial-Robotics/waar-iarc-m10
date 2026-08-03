"""Collision avoidance utilities."""

from __future__ import annotations


class CollisionAvoidance:
    """
    Lightweight collision filtering.

    Prevents multiple drones from occupying
    identical destination blocks.
    """

    def filter_assignments(
        self,
        assignments: dict[
            str,
            tuple[int, int],
        ],
    ) -> dict[str, tuple[int, int]]:

        filtered = {}

        occupied = set()

        for drone_id, block in assignments.items():

            if block in occupied:
                continue

            occupied.add(block)

            filtered[drone_id] = block

        return filtered