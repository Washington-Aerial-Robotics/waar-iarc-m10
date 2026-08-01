"""
test_belief_fusion.py
---------------------
Unit tests for mas_sync/belief_fusion.py

Run from the repo root (no ROS2 needed):
  cd ~/ros2_ws/src/mas_coordinator
  python3 -m pytest tests/test_belief_fusion.py -v
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mas_sync"))

import pytest
import time
from mas_sync.belief_fusion import BeliefEntry, BeliefStore


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_entry(mine_id="m1", x=1.0, y=2.0, confidence=0.5,
               status="candidate", updated_by="d1", seq=1) -> BeliefEntry:
    return BeliefEntry(
        mine_id=mine_id, x=x, y=y,
        confidence=confidence, status=status,
        last_updated_by=updated_by, seq=seq,
        stamp_sec=time.time(),
    )


# ── BeliefStore.merge ─────────────────────────────────────────────────────────

class TestMerge:

    def test_new_entry_is_stored(self):
        store = BeliefStore()
        entry = make_entry()
        changed = store.merge(entry)
        assert changed is True
        assert store.get("m1") is not None

    def test_higher_seq_wins(self):
        store = BeliefStore()
        store.merge(make_entry(confidence=0.9, seq=1))
        changed = store.merge(make_entry(confidence=0.1, seq=2))
        assert changed is True
        assert store.get("m1").confidence == pytest.approx(0.1)

    def test_lower_seq_loses(self):
        store = BeliefStore()
        store.merge(make_entry(confidence=0.5, seq=5))
        changed = store.merge(make_entry(confidence=0.9, seq=3))
        assert changed is False
        assert store.get("m1").confidence == pytest.approx(0.5)

    def test_equal_seq_higher_confidence_wins(self):
        store = BeliefStore()
        store.merge(make_entry(confidence=0.4, seq=2))
        changed = store.merge(make_entry(confidence=0.9, seq=2))
        assert changed is True
        assert store.get("m1").confidence == pytest.approx(0.9)

    def test_equal_seq_lower_confidence_loses(self):
        store = BeliefStore()
        store.merge(make_entry(confidence=0.8, seq=2))
        changed = store.merge(make_entry(confidence=0.3, seq=2))
        assert changed is False
        assert store.get("m1").confidence == pytest.approx(0.8)

    def test_multiple_mines_stored_independently(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", x=1.0))
        store.merge(make_entry(mine_id="m2", x=5.0))
        assert store.count() == 2
        assert store.get("m1").x == pytest.approx(1.0)
        assert store.get("m2").x == pytest.approx(5.0)


# ── Sticky status ─────────────────────────────────────────────────────────────

class TestStickyStatus:

    def test_confirmed_not_downgraded_by_higher_seq(self):
        store = BeliefStore()
        store.merge(make_entry(status="confirmed", seq=2))
        store.merge(make_entry(status="candidate", seq=3))
        # status should remain confirmed
        assert store.get("m1").status == "confirmed"

    def test_rejected_not_downgraded_by_higher_seq(self):
        store = BeliefStore()
        store.merge(make_entry(status="rejected", seq=2))
        store.merge(make_entry(status="candidate", seq=3))
        assert store.get("m1").status == "rejected"

    def test_confirmed_upgrades_to_confirmed(self):
        store = BeliefStore()
        store.merge(make_entry(status="confirmed", seq=2))
        store.merge(make_entry(status="confirmed", confidence=0.99, seq=3))
        assert store.get("m1").status == "confirmed"
        assert store.get("m1").confidence == pytest.approx(0.99)

    def test_candidate_becomes_confirmed(self):
        store = BeliefStore()
        store.merge(make_entry(status="candidate", seq=1))
        store.merge(make_entry(status="confirmed", seq=2))
        assert store.get("m1").status == "confirmed"


# ── merge_batch ───────────────────────────────────────────────────────────────

class TestMergeBatch:

    def test_batch_returns_change_count(self):
        store = BeliefStore()
        entries = [
            make_entry(mine_id="m1", seq=1),
            make_entry(mine_id="m2", seq=1),
            make_entry(mine_id="m3", seq=1),
        ]
        count = store.merge_batch(entries)
        assert count == 3

    def test_batch_deduplicates(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", seq=5))
        entries = [
            make_entry(mine_id="m1", seq=3),   # older → skip
            make_entry(mine_id="m2", seq=1),   # new
        ]
        count = store.merge_batch(entries)
        assert count == 1
        assert store.count() == 2


# ── get_delta_since ───────────────────────────────────────────────────────────

class TestDelta:

    def test_delta_returns_newer_entries(self):
        store = BeliefStore()
        for i in range(5):
            store.merge(make_entry(mine_id=f"m{i}", seq=i+1))
        delta = store.get_delta_since(known_count=3)
        assert len(delta) == 2

    def test_delta_empty_when_fully_synced(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1"))
        delta = store.get_delta_since(known_count=1)
        assert len(delta) == 0


# ── candidates ────────────────────────────────────────────────────────────────

class TestCandidates:

    def test_only_candidates_above_threshold_returned(self):
        store = BeliefStore()
        store.merge(make_entry(mine_id="m1", status="candidate",  confidence=0.8))
        store.merge(make_entry(mine_id="m2", status="candidate",  confidence=0.1))
        store.merge(make_entry(mine_id="m3", status="confirmed",  confidence=0.9))
        candidates = store.candidates(min_confidence=0.3)
        ids = {e.mine_id for e in candidates}
        assert "m1" in ids
        assert "m2" not in ids   # below threshold
        assert "m3" not in ids   # confirmed, not candidate
