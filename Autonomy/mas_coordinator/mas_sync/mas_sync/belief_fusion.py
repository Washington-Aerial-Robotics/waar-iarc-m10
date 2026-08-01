"""
belief_fusion.py
----------------
Pure-logic helper for merging incoming MineBelief updates into the local store.

Rules:
  1. Higher seq always wins (last-write wins per mine_id).
  2. If seqs are equal, higher confidence wins.
  3. "confirmed" and "rejected" are sticky: once set, only a higher-seq update
     can change them.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import math


@dataclass
class BeliefEntry:
    mine_id: str
    x: float
    y: float
    confidence: float
    status: str            # candidate | confirmed | rejected | uncertain
    last_updated_by: str
    seq: int
    stamp_sec: float       # seconds since epoch (for age checks)


class BeliefStore:
    """
    Thread-safe (GIL is enough for Python) in-memory mine belief store.
    """

    STICKY_STATUSES = {"confirmed", "rejected"}

    def __init__(self) -> None:
        # mine_id -> BeliefEntry
        self._store: Dict[str, BeliefEntry] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def merge(self, incoming: BeliefEntry) -> bool:
        """
        Merge one incoming belief.  Returns True if the local store changed.
        """
        existing = self._store.get(incoming.mine_id)

        if existing is None:
            self._store[incoming.mine_id] = incoming
            return True

        # Rule 1: higher seq wins
        if incoming.seq > existing.seq:
            # Rule 3: confirmed is never downgraded to rejected (safety-first).
            # Also preserve sticky status when incoming is a non-sticky downgrade.
            if existing.status == "confirmed" and incoming.status == "rejected":
                incoming.status = "confirmed"
            elif (existing.status in self.STICKY_STATUSES and
                    incoming.status not in self.STICKY_STATUSES):
                incoming.status = existing.status
            self._store[incoming.mine_id] = incoming
            return True

        # Rule 2: equal seq — confirmed > rejected > candidate; then higher confidence
        if incoming.seq == existing.seq:
            # confirmed beats rejected (safety-first: never discard a live mine)
            if (incoming.status == "confirmed"
                    and existing.status == "rejected"):
                self._store[incoming.mine_id] = incoming
                return True
            if incoming.confidence > existing.confidence:
                self._store[incoming.mine_id] = incoming
                return True

        return False

    def merge_batch(self, entries: List[BeliefEntry]) -> int:
        """Merge a list; returns count of changes."""
        return sum(1 for e in entries if self.merge(e))

    def get(self, mine_id: str) -> Optional[BeliefEntry]:
        return self._store.get(mine_id)

    def all(self) -> List[BeliefEntry]:
        return list(self._store.values())

    def get_delta_since(self, known_count: int) -> List[BeliefEntry]:
        """
        Simple delta: return all entries the sender probably doesn't have.
        We use total count as a rough watermark.  For MVP this is fine.
        A proper implementation would track per-peer seq cursors.
        """
        all_entries = self.all()
        # Sort by seq descending so the freshest updates go first
        all_entries.sort(key=lambda e: e.seq, reverse=True)
        return all_entries[known_count:]  # everything beyond what they know

    def count(self) -> int:
        return len(self._store)

    def candidates(self, min_confidence: float = 0.3) -> List[BeliefEntry]:
        """Return mines worth verifying."""
        return [e for e in self._store.values()
                if e.status == "candidate" and e.confidence >= min_confidence]


# ------------------------------------------------------------------
# Utility: convert ROS2 MineBelief msg <-> BeliefEntry
# ------------------------------------------------------------------

def msg_to_entry(msg) -> BeliefEntry:
    """Convert a mas_interfaces/msg/MineBelief ROS2 message to a BeliefEntry."""
    return BeliefEntry(
        mine_id=msg.mine_id,
        x=msg.x,
        y=msg.y,
        confidence=msg.confidence,
        status=msg.status,
        last_updated_by=msg.last_updated_by,
        seq=msg.seq,
        stamp_sec=msg.stamp.sec + msg.stamp.nanosec * 1e-9,
    )


def entry_to_msg(entry: BeliefEntry, mine_belief_cls, time_now):
    """Convert a BeliefEntry back to a MineBelief message."""
    msg = mine_belief_cls()
    msg.mine_id = entry.mine_id
    msg.x = entry.x
    msg.y = entry.y
    msg.confidence = entry.confidence
    msg.status = entry.status
    msg.last_updated_by = entry.last_updated_by
    msg.seq = entry.seq
    msg.stamp = time_now
    return msg
