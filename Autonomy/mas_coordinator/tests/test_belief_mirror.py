"""Regression tests for the mission coordinator's canonical belief mirror."""

import os
import sys
from types import SimpleNamespace

TEST_DIR = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(TEST_DIR, "..", "mas_mission"))
sys.path.insert(0, os.path.join(TEST_DIR, "..", "mas_sync"))

from mas_mission.belief_mirror import BeliefMirror


def make_belief(status="candidate", seq=1, confidence=0.5, mine_id="m1"):
    return SimpleNamespace(
        mine_id=mine_id,
        x=1.0,
        y=2.0,
        confidence=confidence,
        status=status,
        last_updated_by="d1",
        seq=seq,
        stamp=SimpleNamespace(sec=100, nanosec=0),
    )


def make_delta(*beliefs):
    return SimpleNamespace(beliefs=list(beliefs))


def test_rejected_belief_cannot_revert_to_candidate_at_higher_seq():
    mirror = BeliefMirror()
    mirror.merge_delta(make_delta(make_belief(status="rejected", seq=1)))
    mirror.merge_delta(make_delta(make_belief(status="candidate", seq=2)))

    assert mirror.beliefs["m1"]["status"] == "rejected"
    assert mirror.beliefs["m1"]["seq"] == 2


def test_confirmed_belief_cannot_be_downgraded():
    mirror = BeliefMirror()
    mirror.merge_delta(make_delta(make_belief(status="confirmed", seq=1)))
    mirror.merge_delta(make_delta(make_belief(status="rejected", seq=2)))

    assert mirror.beliefs["m1"]["status"] == "confirmed"


def test_rejected_belief_can_be_upgraded_to_confirmed():
    mirror = BeliefMirror()
    mirror.merge_delta(make_delta(make_belief(status="rejected", seq=1)))
    mirror.merge_delta(make_delta(make_belief(status="confirmed", seq=2)))

    assert mirror.beliefs["m1"]["status"] == "confirmed"


def test_invalid_belief_is_not_added_to_mission_mirror():
    mirror = BeliefMirror()
    changed = mirror.merge_delta(
        make_delta(make_belief(status="not-a-status", seq=1)))

    assert changed == 0
    assert mirror.beliefs == {}
