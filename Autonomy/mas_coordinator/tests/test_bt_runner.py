"""
test_bt_runner.py
-----------------
Unit tests for mas_mission/bt_runner.py

Run:
  python3 -m pytest tests/test_bt_runner.py -v
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "mas_mission"))

import time
import pytest
from unittest.mock import Mock, patch

from mas_mission.bt_runner import (
    BTStatus,
    BTNode,
    PrioritySelector,
    CollisionGuardNode,
    GeofenceGuardNode,
    FailureMonitorNode,
    TaskExecutorNode,
    ExplorationPolicyNode,
    P2PSyncManagerNode,
    R_COLLISION,
    POSE_STALE_S,
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_node(
    own_x=5.0, own_y=5.0,
    arena_w=30.0, arena_h=30.0,
    team_poses=None,
    own_pose_last_seen=None,
    pending_task_cmd=None,
    sm_state="SURVEY",
):
    """Return a Mock shaped like MissionLogicNode with sensible defaults."""
    node = Mock()
    node.drone_id            = "d1"
    node.own_x               = own_x
    node.own_y               = own_y
    node.arena_w             = arena_w
    node.arena_h             = arena_h
    node.team_poses          = team_poses if team_poses is not None else {}
    node.own_pose_last_seen  = (
        own_pose_last_seen if own_pose_last_seen is not None
        else time.monotonic()
    )
    node.pending_task_cmd    = pending_task_cmd
    node.sm.state            = sm_state
    return node


def published_cmd(node) -> str:
    """Return the first positional arg of the most recent _publish_cmd call."""
    return node._publish_cmd.call_args[0][0]


# ── PrioritySelector ──────────────────────────────────────────────────────────

class TestPrioritySelector:

    def test_stops_at_first_success_skips_rest(self):
        n1, n2, n3 = Mock(spec=BTNode), Mock(spec=BTNode), Mock(spec=BTNode)
        n1.tick.return_value = BTStatus.SUCCESS
        ctx = make_node()

        result = PrioritySelector([n1, n2, n3]).tick(ctx)

        assert result == BTStatus.SUCCESS
        n1.tick.assert_called_once_with(ctx)
        n2.tick.assert_not_called()
        n3.tick.assert_not_called()

    def test_all_failure_returns_failure_runs_all(self):
        n1, n2 = Mock(spec=BTNode), Mock(spec=BTNode)
        n1.tick.return_value = BTStatus.FAILURE
        n2.tick.return_value = BTStatus.FAILURE
        ctx = make_node()

        result = PrioritySelector([n1, n2]).tick(ctx)

        assert result == BTStatus.FAILURE
        n1.tick.assert_called_once()
        n2.tick.assert_called_once()

    def test_running_stops_selector_returns_running(self):
        n1, n2 = Mock(spec=BTNode), Mock(spec=BTNode)
        n1.tick.return_value = BTStatus.RUNNING
        ctx = make_node()

        result = PrioritySelector([n1, n2]).tick(ctx)

        assert result == BTStatus.RUNNING
        n2.tick.assert_not_called()

    def test_failure_then_success_returns_success(self):
        n1, n2, n3 = Mock(spec=BTNode), Mock(spec=BTNode), Mock(spec=BTNode)
        n1.tick.return_value = BTStatus.FAILURE
        n2.tick.return_value = BTStatus.SUCCESS
        ctx = make_node()

        result = PrioritySelector([n1, n2, n3]).tick(ctx)

        assert result == BTStatus.SUCCESS
        n3.tick.assert_not_called()  # stopped at n2


# ── CollisionGuardNode ────────────────────────────────────────────────────────

class TestCollisionGuardNode:

    def test_success_when_neighbor_within_r_collision(self):
        # Neighbor 0.5 m away; R_COLLISION = 0.8 m
        node = make_node(own_x=5.0, own_y=5.0, team_poses={"d2": (5.5, 5.0)})
        assert CollisionGuardNode().tick(node) == BTStatus.SUCCESS
        assert '"HOLD"' in published_cmd(node)

    def test_failure_when_no_neighbor_within_r_collision(self):
        # Neighbor ~7 m away
        node = make_node(own_x=5.0, own_y=5.0, team_poses={"d2": (10.0, 10.0)})
        assert CollisionGuardNode().tick(node) == BTStatus.FAILURE
        node._publish_cmd.assert_not_called()

    def test_failure_with_no_neighbors(self):
        node = make_node(team_poses={})
        assert CollisionGuardNode().tick(node) == BTStatus.FAILURE

    def test_exactly_at_r_collision_boundary_is_safe(self):
        # dist == R_COLLISION (0.8 m) is NOT < R_COLLISION → FAILURE
        node = make_node(own_x=0.0, own_y=0.0,
                         team_poses={"d2": (R_COLLISION, 0.0)})
        assert CollisionGuardNode().tick(node) == BTStatus.FAILURE

    def test_just_inside_r_collision_boundary_triggers(self):
        # dist = 0.79 m < R_COLLISION (0.8 m) → SUCCESS
        node = make_node(own_x=0.0, own_y=0.0,
                         team_poses={"d2": (R_COLLISION - 0.01, 0.0)})
        assert CollisionGuardNode().tick(node) == BTStatus.SUCCESS


# ── GeofenceGuardNode ─────────────────────────────────────────────────────────

class TestGeofenceGuardNode:

    def test_success_when_x_negative(self):
        node = make_node(own_x=-0.1, own_y=5.0)
        assert GeofenceGuardNode().tick(node) == BTStatus.SUCCESS
        assert '"RETURN_TO_BOUNDS"' in published_cmd(node)

    def test_success_when_x_exceeds_arena_w(self):
        node = make_node(own_x=30.1, own_y=5.0, arena_w=30.0)
        assert GeofenceGuardNode().tick(node) == BTStatus.SUCCESS

    def test_success_when_y_negative(self):
        node = make_node(own_x=5.0, own_y=-0.5)
        assert GeofenceGuardNode().tick(node) == BTStatus.SUCCESS

    def test_success_when_y_exceeds_arena_h(self):
        node = make_node(own_x=5.0, own_y=30.1, arena_h=30.0)
        assert GeofenceGuardNode().tick(node) == BTStatus.SUCCESS

    def test_failure_when_inside_arena(self):
        node = make_node(own_x=5.0, own_y=5.0, arena_w=30.0, arena_h=30.0)
        assert GeofenceGuardNode().tick(node) == BTStatus.FAILURE
        node._publish_cmd.assert_not_called()

    def test_failure_at_origin(self):
        # (0, 0) is on the boundary but NOT < 0, so inside
        node = make_node(own_x=0.0, own_y=0.0)
        assert GeofenceGuardNode().tick(node) == BTStatus.FAILURE


# ── FailureMonitorNode ────────────────────────────────────────────────────────

class TestFailureMonitorNode:

    def test_success_when_pose_is_stale(self):
        node = make_node(own_pose_last_seen=time.monotonic() - (POSE_STALE_S + 1.0))
        assert FailureMonitorNode().tick(node) == BTStatus.SUCCESS
        assert '"FAILSAFE"' in published_cmd(node)

    def test_failure_when_pose_is_fresh(self):
        node = make_node(own_pose_last_seen=time.monotonic() - 0.1)
        assert FailureMonitorNode().tick(node) == BTStatus.FAILURE
        node._publish_cmd.assert_not_called()

    def test_boundary_exactly_at_threshold_is_fresh(self):
        # age == POSE_STALE_S is NOT > POSE_STALE_S → FAILURE
        fixed_now = 1000.0
        node = make_node(own_pose_last_seen=fixed_now - POSE_STALE_S)
        with patch("mas_mission.bt_runner.time.monotonic", return_value=fixed_now):
            result = FailureMonitorNode().tick(node)
        assert result == BTStatus.FAILURE

    def test_one_ms_past_threshold_triggers_failsafe(self):
        fixed_now = 1000.0
        node = make_node(own_pose_last_seen=fixed_now - POSE_STALE_S - 0.001)
        with patch("mas_mission.bt_runner.time.monotonic", return_value=fixed_now):
            result = FailureMonitorNode().tick(node)
        assert result == BTStatus.SUCCESS


# ── TaskExecutorNode ──────────────────────────────────────────────────────────

class TestTaskExecutorNode:

    def test_failure_when_no_pending_task(self):
        node = make_node(pending_task_cmd=None)
        assert TaskExecutorNode().tick(node) == BTStatus.FAILURE
        node._publish_cmd.assert_not_called()

    def test_success_when_task_present(self):
        cmd = '{"task_id": "t1", "task_type": "VERIFY_TAG"}'
        node = make_node(pending_task_cmd=cmd)
        assert TaskExecutorNode().tick(node) == BTStatus.SUCCESS
        node._publish_cmd.assert_called_once_with(cmd)

    def test_clears_pending_task_after_execution(self):
        node = make_node(pending_task_cmd='{"cmd": "go"}')
        TaskExecutorNode().tick(node)
        assert node.pending_task_cmd is None

    def test_does_not_clear_when_no_task(self):
        node = make_node(pending_task_cmd=None)
        TaskExecutorNode().tick(node)
        assert node.pending_task_cmd is None  # stays None, not corrupted


# ── P2PSyncManagerNode ────────────────────────────────────────────────────────

class TestP2PSyncManagerNode:

    def test_always_success(self):
        assert P2PSyncManagerNode().tick(make_node()) == BTStatus.SUCCESS

    def test_publishes_nothing(self):
        node = make_node()
        P2PSyncManagerNode().tick(node)
        node._publish_cmd.assert_not_called()


# ── Priority order integration ────────────────────────────────────────────────

class TestPriorityOrderIntegration:

    def _full_tree(self):
        return PrioritySelector([
            CollisionGuardNode(),
            GeofenceGuardNode(),
            FailureMonitorNode(),
            TaskExecutorNode(),
            ExplorationPolicyNode(),
            P2PSyncManagerNode(),
        ])

    def test_collision_guard_fires_task_executor_never_runs(self):
        """CollisionGuard SUCCESS → TaskExecutor skipped; pending_task_cmd untouched."""
        cmd = '{"task_id": "t1"}'
        node = make_node(
            own_x=0.0, own_y=0.0,
            team_poses={"d2": (0.5, 0.0)},   # 0.5 m < R_COLLISION
            pending_task_cmd=cmd,
            own_pose_last_seen=time.monotonic(),  # fresh
        )

        result = self._full_tree().tick(node)

        assert result == BTStatus.SUCCESS
        assert '"HOLD"' in published_cmd(node)
        assert node.pending_task_cmd == cmd   # not consumed

    def test_geofence_fires_task_executor_never_runs(self):
        """GeofenceGuard SUCCESS → TaskExecutor skipped; pending_task_cmd untouched."""
        cmd = '{"task_id": "t2"}'
        node = make_node(
            own_x=-1.0, own_y=5.0,           # outside left wall
            team_poses={},                     # no collision
            pending_task_cmd=cmd,
            own_pose_last_seen=time.monotonic(),
        )

        result = self._full_tree().tick(node)

        assert result == BTStatus.SUCCESS
        assert '"RETURN_TO_BOUNDS"' in published_cmd(node)
        assert node.pending_task_cmd == cmd   # not consumed

    def test_all_guards_clear_exploration_policy_runs(self):
        """With no hazards and no pending task, ExplorationPolicyNode is reached."""
        node = make_node(
            own_x=5.0, own_y=5.0,
            team_poses={"d2": (20.0, 20.0)},  # ~21 m away, safe
            pending_task_cmd=None,
            own_pose_last_seen=time.monotonic(),
            sm_state="SURVEY",
        )

        result = self._full_tree().tick(node)

        assert result == BTStatus.SUCCESS
        node._cmd_survey.assert_called_once()
