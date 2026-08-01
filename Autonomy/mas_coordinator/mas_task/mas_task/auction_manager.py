"""
auction_manager.py
------------------
Pure-logic auction manager.  No ROS2 imports — easy to unit-test.

Lifecycle of one auction:
  1. OPEN   : announce received, claim window starts
  2. CLOSING: claim window expires, winner selected
  3. DONE   : winner determined (or no bidders → abandoned)

Tie-breaking (deterministic):
  - Lowest cost wins.
  - Equal cost → lexicographically smallest drone_id wins.
    (All drones run the same logic → same winner selected independently.)
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import time


@dataclass
class TaskAnnounceData:
    task_id:       str
    task_type:     str
    announcer_id:  str
    target_x:      float
    target_y:      float
    priority:      float
    claim_window_s: float
    announced_at:  float = field(default_factory=time.monotonic)


@dataclass
class ClaimData:
    task_id:   str
    bidder_id: str
    cost:      float
    received_at: float = field(default_factory=time.monotonic)


class AuctionEntry:
    def __init__(self, announce: TaskAnnounceData):
        self.announce = announce
        self.claims: List[ClaimData] = []
        self.winner: Optional[str] = None
        self.done: bool = False

    @property
    def is_expired(self) -> bool:
        elapsed = time.monotonic() - self.announce.announced_at
        return elapsed >= self.announce.claim_window_s

    def add_claim(self, claim: ClaimData):
        # Deduplicate: one claim per bidder
        existing_ids = {c.bidder_id for c in self.claims}
        if claim.bidder_id not in existing_ids:
            self.claims.append(claim)

    def resolve(self) -> Optional[str]:
        """Select winner. Returns winner drone_id or None if no bids."""
        if self.done:
            return self.winner
        self.done = True
        if not self.claims:
            self.winner = None
            return None
        # Sort: lowest cost first, then alphabetical drone_id for tie-breaking
        best = sorted(self.claims, key=lambda c: (c.cost, c.bidder_id))[0]
        self.winner = best.bidder_id
        return self.winner


class AuctionManager:
    """
    Manages all active auctions for one drone.

    The node calls:
      - on_announce(data)   when a TaskAnnounce arrives
      - on_claim(data)      when a TaskClaim arrives
      - tick()              periodically (e.g. every 0.2 s) to close expired windows
      - pop_won_tasks()     to retrieve tasks this drone won
    """

    def __init__(self, my_drone_id: str):
        self.my_id = my_drone_id
        self._auctions: Dict[str, AuctionEntry] = {}
        self._won: List[TaskAnnounceData] = []       # tasks we won, waiting to be dispatched
        self._abandoned: List[TaskAnnounceData] = [] # tasks with no bidders

    def on_announce(self, data: TaskAnnounceData):
        if data.task_id in self._auctions:
            return   # duplicate announce, ignore
        self._auctions[data.task_id] = AuctionEntry(data)

    def on_claim(self, data: ClaimData):
        entry = self._auctions.get(data.task_id)
        if entry and not entry.done:
            entry.add_claim(data)

    def tick(self) -> List[str]:
        """
        Close any expired auctions. Returns list of task_ids that were resolved.
        If this drone won, appends to self._won.
        """
        resolved = []
        for task_id, entry in list(self._auctions.items()):
            if entry.is_expired and not entry.done:
                winner = entry.resolve()
                resolved.append(task_id)
                if winner == self.my_id:
                    self._won.append(entry.announce)
                elif winner is None:
                    self._abandoned.append(entry.announce)
                # Keep entry so late claims are silently ignored
        return resolved

    def pop_won_tasks(self) -> List[TaskAnnounceData]:
        """Return and clear the list of tasks this drone won."""
        won, self._won = self._won, []
        return won

    def pop_abandoned_tasks(self) -> List[TaskAnnounceData]:
        """Return and clear the list of tasks with no bidders."""
        abandoned, self._abandoned = self._abandoned, []
        return abandoned

    def has_auction(self, task_id: str) -> bool:
        """Return True if task_id is already registered (open or closed)."""
        return task_id in self._auctions

    def compute_cost(self, announce: TaskAnnounceData,
                     my_x: float, my_y: float,
                     my_state: str, busy: bool) -> Optional[float]:
        """
        Compute bid cost for a task.  Lower = more willing to take it.
        Returns None if this drone should NOT bid (e.g. wrong state, too busy).

        Cost formula (MVP):
          base_cost = euclidean distance to target
          + 1000 if busy (already executing a task)
          × (1 / priority)  — higher priority reduces cost
        """
        # Don't bid if we're in FINISH state
        if my_state in ("FINISH", "BOOT"):
            return None

        # Don't bid if we're already executing a high-priority task
        # (busy flag set by the task node)
        if busy and announce.priority < 0.8:
            return None

        dist = ((my_x - announce.target_x) ** 2 +
                (my_y - announce.target_y) ** 2) ** 0.5

        cost = dist
        if busy:
            cost += 1000.0
        if announce.priority > 0:
            cost /= announce.priority   # normalise by priority

        return cost
