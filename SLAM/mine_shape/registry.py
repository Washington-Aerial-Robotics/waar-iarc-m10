from __future__ import annotations

import math

import numpy as np

from apriltag.kalman import TagTrack
from apriltag.models import FusedMine


class ShapeMineRegistry:
    """Fuse shape-only mine hypotheses (negative shape_id keys)."""

    def __init__(
        self,
        *,
        min_confidence: float = 0.35,
        fusion_radius_m: float = 0.5,
        use_kalman: bool = True,
    ):
        self.min_confidence = min_confidence
        self.fusion_radius_m = fusion_radius_m
        self.use_kalman = use_kalman
        self._mines: dict[int, FusedMine] = {}
        self._tracks: dict[int, TagTrack] = {}
        self._next_shape_id = -1

    @property
    def mines(self) -> dict[int, FusedMine]:
        return dict(self._mines)

    def _alloc_id(self) -> int:
        sid = self._next_shape_id
        self._next_shape_id -= 1
        return sid

    def _find_near(self, world_position: np.ndarray) -> int | None:
        for sid, mine in self._mines.items():
            d = math.hypot(
                float(mine.world_position[0]) - float(world_position[0]),
                float(mine.world_position[1]) - float(world_position[1]),
            )
            if d < self.fusion_radius_m:
                return sid
        return None

    def update(
        self,
        world_position: np.ndarray,
        confidence: float,
        timestamp: float,
    ) -> FusedMine | None:
        if confidence < self.min_confidence:
            return None

        world_position = world_position.astype(np.float64).reshape(3)
        near = self._find_near(world_position)

        if self.use_kalman:
            track_key = near if near is not None else self._next_shape_id
            if track_key not in self._tracks:
                self._tracks[track_key] = TagTrack(world_position, timestamp)
            world_position = self._tracks[track_key].update(world_position, timestamp)

        if near is None:
            sid = self._alloc_id()
            fused = FusedMine(
                tag_id=None,
                shape_id=sid,
                source="shape",
                first_seen=timestamp,
                last_seen=timestamp,
                observation_count=1,
                world_position=world_position.copy(),
                confidence=float(confidence),
                world_rotation=None,
            )
            self._mines[sid] = fused
            return fused

        existing = self._mines[near]
        old_weight = existing.confidence * existing.observation_count
        new_weight = confidence
        fused_position = (existing.world_position * old_weight + world_position * new_weight) / (
            old_weight + new_weight
        )
        existing.last_seen = timestamp
        existing.observation_count += 1
        existing.world_position = fused_position
        existing.confidence = min(1.0, (existing.confidence + confidence) / 2.0)
        return existing

    def remove_near(self, world_position: np.ndarray, radius_m: float) -> int:
        removed = 0
        to_drop: list[int] = []
        for sid, mine in self._mines.items():
            d = math.hypot(
                float(mine.world_position[0]) - float(world_position[0]),
                float(mine.world_position[1]) - float(world_position[1]),
            )
            if d < radius_m:
                to_drop.append(sid)
        for sid in to_drop:
            del self._mines[sid]
            self._tracks.pop(sid, None)
            removed += 1
        return removed
