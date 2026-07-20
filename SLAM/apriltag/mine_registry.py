from __future__ import annotations

import numpy as np

from .kalman import TagTrack
from .models import FusedMine


class MineRegistry:
    """Fuse repeated observations of the same tag ID into one mine estimate."""

    def __init__(self, min_confidence: float = 0.1, use_kalman: bool = True):
        self.min_confidence = min_confidence
        self.use_kalman = use_kalman
        self._mines: dict[int, FusedMine] = {}
        self._tracks: dict[int, TagTrack] = {}

    @property
    def mines(self) -> dict[int, FusedMine]:
        return dict(self._mines)

    def update(
        self,
        tag_id: int,
        world_position: np.ndarray,
        world_rotation: np.ndarray | None,
        confidence: float,
        timestamp: float,
    ) -> FusedMine | None:
        if confidence < self.min_confidence:
            return None

        world_position = world_position.astype(np.float64).reshape(3)

        if self.use_kalman:
            if tag_id not in self._tracks:
                self._tracks[tag_id] = TagTrack(world_position, timestamp)
            track = self._tracks[tag_id]
            world_position = track.update(world_position, timestamp)

        if tag_id not in self._mines:
            fused = FusedMine(
                tag_id=tag_id,
                first_seen=timestamp,
                last_seen=timestamp,
                observation_count=1,
                world_position=world_position.copy(),
                confidence=float(confidence),
                world_rotation=None if world_rotation is None else world_rotation.copy(),
            )
            self._mines[tag_id] = fused
            return fused

        existing = self._mines[tag_id]
        old_weight = existing.confidence * existing.observation_count
        new_weight = confidence
        total_weight = old_weight + new_weight

        fused_position = (
            existing.world_position * old_weight + world_position * new_weight
        ) / total_weight

        fused_confidence = min(1.0, (existing.confidence + confidence) / 2.0)

        existing.last_seen = timestamp
        existing.observation_count += 1
        existing.world_position = fused_position
        existing.confidence = fused_confidence
        if world_rotation is not None:
            existing.world_rotation = world_rotation.copy()

        return existing
