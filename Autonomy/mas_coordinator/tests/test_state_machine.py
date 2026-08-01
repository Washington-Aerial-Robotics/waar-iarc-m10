"""
test_state_machine.py
---------------------
Unit tests for mas_mission/state_machine.py

Run:
  python3 -m pytest tests/test_state_machine.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mas_mission"))

import pytest
import time
from mas_mission.state_machine import (
    StateMachine, MissionContext,
    T_PATH_VERIFY, T_CONVERGE, T_FINISH,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_ctx(**kwargs) -> MissionContext:
    defaults = dict(
        time_remaining=600.0,
        mine_count=0,
        confirmed_count=0,
        all_drones_ready=False,
        path_verified=False,
        all_converged=False,
        team_belief_count=0,
    )
    defaults.update(kwargs)
    return MissionContext(**defaults)


# ── BOOT → SURVEY ─────────────────────────────────────────────────────────────

class TestBootToSurvey:

    def test_stays_in_boot_without_ready(self):
        sm = StateMachine("d1")
        sm.tick(make_ctx(all_drones_ready=False))
        assert sm.state == "BOOT"

    def test_transitions_to_survey_when_ready(self):
        sm = StateMachine("d1")
        sm.tick(make_ctx(all_drones_ready=True))
        assert sm.state == "SURVEY"


# ── SURVEY transitions ────────────────────────────────────────────────────────

class TestSurveyTransitions:

    def setup_method(self):
        self.sm = StateMachine("d1")
        self.sm._go("SURVEY")

    def test_survey_to_verify_tag_on_mine(self):
        self.sm.tick(make_ctx(mine_count=1, time_remaining=300.0))
        assert self.sm.state == "VERIFY_TAG"

    def test_survey_to_path_verify_on_time(self):
        self.sm.tick(make_ctx(time_remaining=T_PATH_VERIFY - 1))
        assert self.sm.state == "PATH_VERIFY"

    def test_survey_stays_with_no_mines_and_time(self):
        self.sm.tick(make_ctx(mine_count=0, time_remaining=300.0))
        assert self.sm.state == "SURVEY"

    def test_time_override_beats_mine_condition(self):
        # Even with mines, if time is low go to PATH_VERIFY not VERIFY_TAG
        self.sm.tick(make_ctx(mine_count=5, time_remaining=T_PATH_VERIFY - 1))
        assert self.sm.state == "PATH_VERIFY"


# ── VERIFY_TAG transitions ────────────────────────────────────────────────────

class TestVerifyTagTransitions:

    def setup_method(self):
        self.sm = StateMachine("d1")
        self.sm._go("VERIFY_TAG")

    def test_verify_to_survey_when_all_confirmed(self):
        self.sm.tick(make_ctx(mine_count=3, confirmed_count=3, time_remaining=300.0))
        assert self.sm.state == "SURVEY"

    def test_verify_to_path_verify_on_time(self):
        self.sm.tick(make_ctx(mine_count=1, confirmed_count=0,
                              time_remaining=T_PATH_VERIFY - 1))
        assert self.sm.state == "PATH_VERIFY"

    def test_verify_stays_with_unresolved_mines(self):
        self.sm.tick(make_ctx(mine_count=3, confirmed_count=1, time_remaining=300.0))
        assert self.sm.state == "VERIFY_TAG"


# ── PATH_VERIFY transitions ───────────────────────────────────────────────────

class TestPathVerifyTransitions:

    def setup_method(self):
        self.sm = StateMachine("d1")
        self.sm._go("PATH_VERIFY")

    def test_path_verify_to_converge_on_time(self):
        self.sm.tick(make_ctx(time_remaining=T_CONVERGE - 1))
        assert self.sm.state == "CONVERGE"

    def test_path_verify_to_converge_when_verified(self):
        self.sm.tick(make_ctx(path_verified=True, time_remaining=300.0))
        assert self.sm.state == "CONVERGE"

    def test_path_verify_stays_without_trigger(self):
        self.sm.tick(make_ctx(path_verified=False, time_remaining=300.0))
        assert self.sm.state == "PATH_VERIFY"


# ── CONVERGE transitions ──────────────────────────────────────────────────────

class TestConvergeTransitions:

    def setup_method(self):
        self.sm = StateMachine("d1")
        self.sm._go("CONVERGE")

    def test_converge_to_finish_on_time(self):
        self.sm.tick(make_ctx(time_remaining=T_FINISH - 1))
        assert self.sm.state == "FINISH"

    def test_converge_to_finish_when_all_converged(self):
        self.sm.tick(make_ctx(all_converged=True, time_remaining=300.0))
        assert self.sm.state == "FINISH"

    def test_converge_stays_without_trigger(self):
        self.sm.tick(make_ctx(all_converged=False, time_remaining=300.0))
        assert self.sm.state == "CONVERGE"


# ── FINISH is terminal ────────────────────────────────────────────────────────

class TestFinish:

    def test_finish_is_terminal(self):
        sm = StateMachine("d1")
        sm._go("FINISH")
        sm.tick(make_ctx(all_drones_ready=True, time_remaining=0.0))
        assert sm.state == "FINISH"
        assert sm.is_terminal() is True


# ── Transition callback ───────────────────────────────────────────────────────

class TestTransitionCallback:

    def test_callback_fired_on_transition(self):
        transitions = []
        sm = StateMachine("d1")
        sm.on_transition(lambda f, t: transitions.append((f, t)))
        sm.tick(make_ctx(all_drones_ready=True))
        assert transitions == [("BOOT", "SURVEY")]

    def test_callback_not_fired_without_transition(self):
        transitions = []
        sm = StateMachine("d1")
        sm.on_transition(lambda f, t: transitions.append((f, t)))
        sm.tick(make_ctx(all_drones_ready=False))
        assert transitions == []


# ── ros2_adapter coordinate conversion (no ROS2 needed) ──────────────────────

class TestCoordConversion:
    """
    Tests the pure-math coord_to_xy / xy_to_coord functions from ros2_adapter.
    Import them directly without instantiating the ROS2 node.
    """

    def _load(self):
        adapter_path = os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "waar_autonomy", "src", "adapters"
        )
        if os.path.exists(adapter_path):
            sys.path.insert(0, adapter_path)
            from ros2_adapter import coord_to_xy, xy_to_coord
            return coord_to_xy, xy_to_coord
        else:
            pytest.skip("waar_autonomy not found — skipping adapter tests")

    def test_coord_to_xy_origin(self):
        coord_to_xy, _ = self._load()
        x, y = coord_to_xy((0, 0))
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)

    def test_coord_to_xy_basic(self):
        coord_to_xy, _ = self._load()
        # (row=2, col=3) with CELL_SIZE=1.0 → x=3.0, y=2.0
        x, y = coord_to_xy((2, 3))
        assert x == pytest.approx(3.0)
        assert y == pytest.approx(2.0)

    def test_xy_to_coord_basic(self):
        _, xy_to_coord = self._load()
        row, col = xy_to_coord(3.0, 2.0)
        assert row == 2
        assert col == 3

    def test_roundtrip(self):
        coord_to_xy, xy_to_coord = self._load()
        original = (4, 7)
        x, y = coord_to_xy(original)
        recovered = xy_to_coord(x, y)
        assert recovered == original
