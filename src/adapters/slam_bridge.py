"""
Bridge SLAM mine registry output into the multi-agent simulator.

Example:
    from SLAM.apriltag import MineRegistry
    from src.adapters.slam_bridge import mines_for_coordinator

    fused = registry.mines
    coordinator.apply_external_mines(mines_for_coordinator(fused.values()))
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
