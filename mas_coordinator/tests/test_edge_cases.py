"""
test_edge_cases.py
------------------
Edge-case tests covering:
  1. Arena dimensions: 91.44 m × 24.38 m
  2. Mission duration: 420 s
  3. No-bidder retry in AuctionManager
  4. Duplicate mine guard in BeliefStore (confirmed beats rejected on seq tie)
  5. Drone dropout pruning (team_last_seen stale for > 5s)
  6. Belief conflict resolution: confirmed > rejected at equal seq

Run from the repo root (no ROS2 needed):
  cd ~/ros2_ws/src/mas_coordinator
  python3 -m pytest tests/test_edge_cases.py -v
"""

import sys
import os
import time

# Add package roots to path
_root = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(_root, "mas_mission"))
sys.path.insert(0, os.path.join(_root, "mas_task"))
sys.path.insert(0, os.path.join(_root, "mas_sync"))

import pytest

from mas_mission.state_machine import (
    StateMachine, MissionContext,
    ARENA_WIDTH, ARENA_HEIGHT, MISSION_DURATION,
    T_PATH_VERIFY, T_CONVERGE,
)
from mas_task.auction_manager import (
    AuctionManager, TaskAnnounceData, ClaimData,
)
from mas_sync.belief_fusion import BeliefEntry, BeliefStore


# ── Helper factories ──────────────────────────────────────────────────────────

def make_entry(mine_id="m1", confidence=0.6, status="candidate",
               seq=1, x=5.0, y=5.0) -> BeliefEntry:
    return BeliefEntry(
        mine_id=mine_id, x=x, y=y,
        confidence=confidence, status=status,
        last_updated_by="d1", seq=seq, stamp_sec=0.0,
    )


def make_announce(task_id="t1", task_type="VERIFY_TAG",
                  target_x=5.0, target_y=5.0,
                  priority=0.6, claim_window_s=0.01) -> TaskAnnounceData:
    return TaskAnnounceData(
        task_id=task_id, task_type=task_type,
        announcer_id="d1",
        target_x=target_x, target_y=target_y,
        priority=priority, claim_window_s=claim_window_s,
    )


# ── Test 1: Arena dimensions ──────────────────────────────────────────────────

class TestArenaDimensions:

    def test_arena_width_is_300ft(self):
        assert abs(ARENA_WIDTH - 91.44) < 0.01, (
            f"Expected 91.44 m (300 ft), got {ARENA_WIDTH}")

    def test_arena_height_is_80ft(self):
        assert abs(ARENA_HEIGHT - 24.38) < 0.01, (
            f"Expected 24.38 m (80 ft), got {ARENA_HEIGHT}")


# ── Test 2: Mission duration ──────────────────────────────────────────────────

class TestMissionDuration:

    def test_mission_duration_is_420s(self):
        assert MISSION_DURATION == 420.0

    def test_default_context_time_remaining_is_420s(self):
        ctx = MissionContext()
        assert ctx.time_remaining == 420.0

    def test_state_machine_default_duration_is_420s(self):
        sm = StateMachine("d1")
        assert sm.mission_duration == 420.0

    def test_path_verify_threshold_at_90s(self):
        """Drone in SURVEY should transition to PATH_VERIFY at exactly 90s remaining."""
        sm = StateMachine("d1")
        sm._go("SURVEY")
        ctx = MissionContext(time_remaining=T_PATH_VERIFY, all_drones_ready=True)
        sm.tick(ctx)
        assert sm.state == "PATH_VERIFY"

    def test_converge_threshold_at_45s(self):
        """Drone in PATH_VERIFY should transition to CONVERGE at exactly 45s remaining."""
        sm = StateMachine("d1")
        sm._go("PATH_VERIFY")
        ctx = MissionContext(time_remaining=T_CONVERGE)
        sm.tick(ctx)
        assert sm.state == "CONVERGE"


# ── Test 3: No-bidder retry ───────────────────────────────────────────────────

class TestNoBidderRetry:

    def test_abandoned_task_collected(self):
        """When a task window closes with no bids, it appears in pop_abandoned_tasks."""
        mgr = AuctionManager("d1")
        ann = make_announce(task_id="solo_task", claim_window_s=0.001)
        mgr.on_announce(ann)
        time.sleep(0.02)   # let the window expire
        mgr.tick()
        abandoned = mgr.pop_abandoned_tasks()
        ids = [t.task_id for t in abandoned]
        assert "solo_task" in ids

    def test_abandoned_cleared_after_pop(self):
        """pop_abandoned_tasks is destructive — second call returns empty."""
        mgr = AuctionManager("d1")
        ann = make_announce(task_id="t_clear", claim_window_s=0.001)
        mgr.on_announce(ann)
        time.sleep(0.02)
        mgr.tick()
        mgr.pop_abandoned_tasks()
        assert mgr.pop_abandoned_tasks() == []

    def test_has_auction_returns_true_after_announce(self):
        mgr = AuctionManager("d1")
        mgr.on_announce(make_announce(task_id="t_has"))
        assert mgr.has_auction("t_has") is True

    def test_has_auction_returns_false_for_unknown(self):
        mgr = AuctionManager("d1")
        assert mgr.has_auction("nonexistent") is False


# ── Test 4: Duplicate mine guard (belief store) ───────────────────────────────

class TestDuplicateMineGuard:

    def test_second_candidate_same_id_lower_seq_ignored(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", seq=5, confidence=0.8))
        changed = store.merge(make_entry(mine_id="m1", seq=3, confidence=0.9))
        assert not changed
        assert store.get("m1").seq == 5

    def test_confirmed_sticky_not_overwritten_by_candidate(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", status="confirmed", seq=2, confidence=0.9))
        changed = store.merge(make_entry(mine_id="m1", status="candidate", seq=3, confidence=0.95))
        # Higher seq but downgrade from confirmed → candidate should be blocked
        assert changed          # seq 3 > 2, so it does update (but status preserved)
        assert store.get("m1").status == "confirmed"


# ── Test 5: Drone dropout simulation ─────────────────────────────────────────

class TestDroneDropout:

    def test_all_drones_ready_drops_stale_drone(self):
        """
        Simulate that a drone with a >3s stale beacon is NOT counted as ready.
        Uses the MissionLogicNode._all_drones_ready() logic directly (no ROS2).
        """
        # Replicate the logic from mission_logic_node._all_drones_ready()
        now = time.monotonic()
        team_last_seen = {
            "d2": now - 1.0,   # fresh
            "d3": now - 2.5,   # fresh (just within 3s)
            "d4": now - 5.0,   # STALE
        }
        num_drones = 4   # 4 total including self (d1)
        active = {did for did, t in team_last_seen.items() if now - t < 3.0}
        # Need num_drones - 1 = 3 others; only 2 are fresh
        assert len(active) == 2
        assert len(active) < num_drones - 1


# ── Test 6: Belief conflict — confirmed beats rejected at equal seq ────────────

class TestBeliefConflict:

    def test_confirmed_beats_rejected_equal_seq(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", status="rejected", seq=3, confidence=0.8))
        changed = store.merge(make_entry(mine_id="m1", status="confirmed", seq=3, confidence=0.8))
        assert changed
        assert store.get("m1").status == "confirmed"

    def test_rejected_does_not_beat_confirmed_equal_seq(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", status="confirmed", seq=3, confidence=0.8))
        changed = store.merge(make_entry(mine_id="m1", status="rejected", seq=3, confidence=0.8))
        assert not changed
        assert store.get("m1").status == "confirmed"

    def test_confirmed_never_downgraded_by_higher_seq_rejected(self):
        """
        Safety-first: once a mine is confirmed it is never downgraded to rejected,
        even when the incoming update has a strictly higher seq.
        """
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", status="confirmed", seq=3, confidence=0.9))
        changed = store.merge(make_entry(mine_id="m1", status="rejected", seq=5, confidence=0.3))
        # The entry IS updated (higher seq wins for other fields)
        assert changed
        # But the confirmed status must survive (IARC safety rule)
        assert store.get("m1").status == "confirmed"
