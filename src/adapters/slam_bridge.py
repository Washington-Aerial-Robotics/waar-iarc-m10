"""
Bridge SLAM mine registry output into the multi-agent simulator.

Example:
    from SLAM.apriltag import MineRegistry
    from src.adapters.slam_bridge import mines_for_coordinator, obstacles_for_coordinator

    fused = registry.mines
    coordinator.apply_external_mines(mines_for_coordinator(fused.values()))
    coordinator.apply_external_obstacles(obstacles_for_coordinator(obstacle_registry.obstacles.values()))
"""

from __future__ import annotations

from typing import Iterable


def mines_for_coordinator(fused_mines: Iterable) -> list[tuple[int, float, float, float]]:
    """Convert fused mine objects to coordinator tuples."""
    out: list[tuple[int, float, float, float]] = []
    for mine in fused_mines:
        pos = mine.world_position
        out.append((mine.tag_id, float(pos[0]), float(pos[1]), float(mine.confidence)))
    return out


def obstacles_for_coordinator(
    fused_obstacles: Iterable,
) -> list[tuple[int, str, float, float, float, float]]:
    """Convert fused obstacle objects to coordinator tuples (not used for human path)."""
    out: list[tuple[int, str, float, float, float, float]] = []
    for obstacle in fused_obstacles:
        pos = obstacle.world_position
        out.append(
            (
                obstacle.obstacle_id,
                obstacle.label,
                float(pos[0]),
                float(pos[1]),
                float(obstacle.confidence),
                float(obstacle.radius_m),
            )
        )
    return out
