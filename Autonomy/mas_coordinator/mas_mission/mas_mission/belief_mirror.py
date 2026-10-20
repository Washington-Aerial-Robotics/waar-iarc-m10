"""Canonical mine-belief mirror used by the mission coordinator."""

from __future__ import annotations

from typing import Dict

from mas_sync.belief_fusion import BeliefStore, msg_to_entry


class BeliefMirror:
    """Expose canonical ``BeliefStore`` state in the mission's dict format."""

    def __init__(self) -> None:
        self._store = BeliefStore()
        self.beliefs: Dict[str, dict] = {}

    def merge(self, belief_msg) -> bool:
        """Merge one ROS-like belief message and refresh the public mirror."""
        incoming = msg_to_entry(belief_msg)
        if not self._store.merge(incoming):
            return False

        stored = self._store.get(incoming.mine_id)
        self.beliefs[stored.mine_id] = {
            "mine_id": stored.mine_id,
            "x": stored.x,
            "y": stored.y,
            "confidence": stored.confidence,
            "status": stored.status,
            "seq": stored.seq,
        }
        return True

    def merge_delta(self, delta_msg) -> int:
        """Merge every belief in a ROS-like MineDelta message."""
        return sum(1 for belief in delta_msg.beliefs if self.merge(belief))
