"""
test_auction_manager.py
-----------------------
Unit tests for mas_task/auction_manager.py

Run:
  python3 -m pytest tests/test_auction_manager.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mas_task"))

import pytest
import time
from unittest.mock import patch
from mas_task.auction_manager import (
    AuctionManager, AuctionEntry,
    TaskAnnounceData, ClaimData,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_announce(task_id="t1", task_type="VERIFY_TAG",
                  announcer="d1", tx=5.0, ty=5.0,
                  priority=0.5, window=2.0) -> TaskAnnounceData:
    return TaskAnnounceData(
        task_id=task_id, task_type=task_type,
        announcer_id=announcer,
        target_x=tx, target_y=ty,
        priority=priority,
        claim_window_s=window,
    )

def make_claim(task_id="t1", bidder="d2", cost=3.0) -> ClaimData:
    return ClaimData(task_id=task_id, bidder_id=bidder, cost=cost)


# ── AuctionEntry ──────────────────────────────────────────────────────────────

class TestAuctionEntry:

    def test_winner_is_lowest_cost(self):
        entry = AuctionEntry(make_announce())
        entry.add_claim(make_claim(bidder="d2", cost=5.0))
        entry.add_claim(make_claim(bidder="d3", cost=2.0))
        entry.add_claim(make_claim(bidder="d4", cost=8.0))
        winner = entry.resolve()
        assert winner == "d3"

    def test_tie_broken_by_drone_id_alphabetical(self):
        entry = AuctionEntry(make_announce())
        entry.add_claim(make_claim(bidder="d3", cost=3.0))
        entry.add_claim(make_claim(bidder="d1", cost=3.0))
        entry.add_claim(make_claim(bidder="d2", cost=3.0))
        winner = entry.resolve()
        assert winner == "d1"   # lexicographically smallest

    def test_no_bids_returns_none(self):
        entry = AuctionEntry(make_announce())
        assert entry.resolve() is None

    def test_duplicate_bids_ignored(self):
        entry = AuctionEntry(make_announce())
        entry.add_claim(make_claim(bidder="d2", cost=5.0))
        entry.add_claim(make_claim(bidder="d2", cost=1.0))  # duplicate, ignored
        assert len(entry.claims) == 1
        assert entry.claims[0].cost == pytest.approx(5.0)

    def test_resolve_is_idempotent(self):
        entry = AuctionEntry(make_announce())
        entry.add_claim(make_claim(bidder="d2", cost=3.0))
        w1 = entry.resolve()
        w2 = entry.resolve()
        assert w1 == w2


# ── AuctionManager ────────────────────────────────────────────────────────────

class TestAuctionManager:

    def test_winner_added_to_won_list(self):
        mgr = AuctionManager("d2")
        announce = make_announce(window=0.01)   # very short window
        mgr.on_announce(announce)
        mgr.on_claim(make_claim(bidder="d2", cost=1.0))
        time.sleep(0.05)
        mgr.tick()
        won = mgr.pop_won_tasks()
        assert len(won) == 1
        assert won[0].task_id == "t1"

    def test_loser_not_in_won_list(self):
        mgr = AuctionManager("d4")
        announce = make_announce(window=0.01)
        mgr.on_announce(announce)
        mgr.on_claim(make_claim(bidder="d2", cost=1.0))  # d2 wins
        mgr.on_claim(make_claim(bidder="d4", cost=9.0))  # d4 loses
        time.sleep(0.05)
        mgr.tick()
        won = mgr.pop_won_tasks()
        assert len(won) == 0

    def test_duplicate_announce_ignored(self):
        mgr = AuctionManager("d1")
        mgr.on_announce(make_announce(task_id="t1"))
        mgr.on_announce(make_announce(task_id="t1"))  # duplicate
        assert len(mgr._auctions) == 1

    def test_pop_clears_won_list(self):
        mgr = AuctionManager("d1")
        announce = make_announce(window=0.01)
        mgr.on_announce(announce)
        mgr.on_claim(make_claim(bidder="d1", cost=1.0))
        time.sleep(0.05)
        mgr.tick()
        mgr.pop_won_tasks()
        assert mgr.pop_won_tasks() == []

    def test_tick_only_resolves_expired(self):
        mgr = AuctionManager("d1")
        mgr.on_announce(make_announce(task_id="t1", window=100.0))  # long window
        mgr.on_claim(make_claim(task_id="t1", bidder="d1", cost=1.0))
        mgr.tick()
        assert mgr.pop_won_tasks() == []   # not expired yet


# ── compute_cost ──────────────────────────────────────────────────────────────

class TestComputeCost:

    def setup_method(self):
        self.mgr = AuctionManager("d1")
        self.announce = make_announce(tx=10.0, ty=0.0, priority=0.5)

    def test_cost_is_distance_based(self):
        # drone at (0,0), target at (10,0) → dist=10
        cost = self.mgr.compute_cost(self.announce, 0.0, 0.0, "SURVEY", False)
        assert cost == pytest.approx(10.0 / 0.5)   # dist / priority

    def test_busy_drone_gets_penalty(self):
        high_priority = make_announce(tx=10.0, ty=0.0, priority=0.9)
        cost_free = self.mgr.compute_cost(high_priority, 0.0, 0.0, "SURVEY", False)
        cost_busy = self.mgr.compute_cost(high_priority, 0.0, 0.0, "SURVEY", True)
        assert cost_busy is not None
        assert cost_busy > cost_free

    def test_finish_state_returns_none(self):
        cost = self.mgr.compute_cost(self.announce, 0.0, 0.0, "FINISH", False)
        assert cost is None

    def test_boot_state_returns_none(self):
        cost = self.mgr.compute_cost(self.announce, 0.0, 0.0, "BOOT", False)
        assert cost is None

    def test_busy_low_priority_returns_none(self):
        low_priority = make_announce(priority=0.3)
        cost = self.mgr.compute_cost(low_priority, 0.0, 0.0, "SURVEY", True)
        assert cost is None

    def test_busy_high_priority_still_bids(self):
        high_priority = make_announce(priority=0.9)
        cost = self.mgr.compute_cost(high_priority, 0.0, 0.0, "SURVEY", True)
        assert cost is not None
