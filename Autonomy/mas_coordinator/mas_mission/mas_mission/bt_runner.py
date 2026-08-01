"""
bt_runner.py
------------
Lightweight Behaviour Tree (BT) framework for mas_mission.

Classes
-------
BTStatus           – tick return value (SUCCESS / FAILURE / RUNNING)
BTNode             – abstract base; subclasses implement tick(node)
PrioritySelector   – run children left-to-right, stop at first non-FAILURE

Guard / Leaf nodes (each receives the MissionLogicNode instance as context):
  CollisionGuardNode    – HOLD when any neighbour is within r_collision metres
  GeofenceGuardNode     – RETURN_TO_BOUNDS when self leaves the arena rectangle
  FailureMonitorNode    – FAILSAFE when own pose is stale (> POSE_STALE_S seconds)
  TaskExecutorNode      – consume a pending task_cmd from the p2p task node
  ExplorationPolicyNode – dispatch per-state mission directive (mirrors old _tick())
  P2PSyncManagerNode    – passive leaf; always returns SUCCESS

Priority order (highest first):
  1. CollisionGuardNode
  2. GeofenceGuardNode
  3. FailureMonitorNode
  4. TaskExecutorNode
  5. ExplorationPolicyNode
  6. P2PSyncManagerNode
"""

from __future__ import annotations

import json
import math
import time  # used by FailureMonitorNode
from enum import Enum, auto
from typing import Any, List


# ── Status ────────────────────────────────────────────────────────────────────

class BTStatus(Enum):
    SUCCESS  = auto()
    FAILURE  = auto()
    RUNNING  = auto()


# ── Base class ────────────────────────────────────────────────────────────────

class BTNode:
    """Abstract BT node.  Subclasses must implement tick(node) -> BTStatus."""

    def tick(self, node: Any) -> BTStatus:
        raise NotImplementedError(f"{type(self).__name__}.tick() not implemented")


# ── Composite ─────────────────────────────────────────────────────────────────

class PrioritySelector(BTNode):
    """
    Priority (fallback) selector composite.

    Runs children left-to-right and returns the first result that is not
    FAILURE.  Only returns FAILURE when every child returns FAILURE.
    A child returning RUNNING stops evaluation and propagates RUNNING up.
    """

    def __init__(self, children: List[BTNode]) -> None:
        self.children = children

    def tick(self, node: Any) -> BTStatus:
        for child in self.children:
            status = child.tick(node)
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE


# ── Constants ─────────────────────────────────────────────────────────────────

R_COLLISION  = 0.8   # metres – minimum safe separation between drones
POSE_STALE_S = 3.0   # seconds – own-pose age before declaring failure


# ── Guard nodes ───────────────────────────────────────────────────────────────

class CollisionGuardNode(BTNode):
    """
    Highest-priority safety guard.

    Compares this drone's own position against every known neighbour pose.
    If any neighbour is closer than R_COLLISION metres:
      • publishes cmd=HOLD to the explorer
      • returns SUCCESS  (selector stops; lower nodes are skipped)
    Returns FAILURE when no collision risk exists.
    """

    def tick(self, node: Any) -> BTStatus:
        ox, oy = node.own_x, node.own_y
        for did, (nx, ny) in node.team_poses.items():
            dist = math.hypot(nx - ox, ny - oy)
            if dist < R_COLLISION:
                node.get_logger().warn(
                    f"[{node.drone_id}] COLLISION risk with {did} "
                    f"dist={dist:.2f}m — issuing HOLD")
                node._publish_cmd(json.dumps({"cmd": "HOLD"}))
                return BTStatus.SUCCESS
        return BTStatus.FAILURE


class GeofenceGuardNode(BTNode):
    """
    Geofence guard.

    Checks whether this drone's position lies within the arena rectangle
    [0, arena_w] × [0, arena_h].  If outside:
      • publishes cmd=RETURN_TO_BOUNDS to the explorer
      • returns SUCCESS
    Returns FAILURE when inside the fence.
    """

    def tick(self, node: Any) -> BTStatus:
        x, y = node.own_x, node.own_y
        if x < 0.0 or x > node.arena_w or y < 0.0 or y > node.arena_h:
            node.get_logger().warn(
                f"[{node.drone_id}] Outside geofence "
                f"pos=({x:.1f},{y:.1f}) arena=({node.arena_w},{node.arena_h})"
                f" — issuing RETURN_TO_BOUNDS")
            node._publish_cmd(json.dumps({"cmd": "RETURN_TO_BOUNDS"}))
            return BTStatus.SUCCESS
        return BTStatus.FAILURE


class FailureMonitorNode(BTNode):
    """
    Detects a potentially-dead drone by watching own-pose freshness.

    If the last self-pose beacon is older than POSE_STALE_S seconds the
    position sensor may have failed:
      • publishes cmd=FAILSAFE to the explorer
      • returns SUCCESS
    Returns FAILURE when the pose is fresh.
    """

    def tick(self, node: Any) -> BTStatus:
        age = time.monotonic() - node.own_pose_last_seen
        if age > POSE_STALE_S:
            node.get_logger().error(
                f"[{node.drone_id}] Own pose stale ({age:.1f}s > {POSE_STALE_S}s)"
                f" — issuing FAILSAFE")
            node._publish_cmd(json.dumps({"cmd": "FAILSAFE"}))
            return BTStatus.SUCCESS
        return BTStatus.FAILURE


# ── Task executor ─────────────────────────────────────────────────────────────

class TaskExecutorNode(BTNode):
    """
    Consume a pending task_cmd delivered by the p2p task node.

    The mission_logic_node buffers the latest JSON string arriving on
    /{drone_id}/task_cmd in node.pending_task_cmd.  When a command is
    waiting this node forwards it to the explorer and clears the buffer,
    then returns SUCCESS so the selector stops.
    Returns FAILURE when the buffer is empty (normal operation).
    """

    def tick(self, node: Any) -> BTStatus:
        if node.pending_task_cmd is not None:
            node._publish_cmd(node.pending_task_cmd)
            node.pending_task_cmd = None
            return BTStatus.SUCCESS
        return BTStatus.FAILURE


# ── Exploration policy ────────────────────────────────────────────────────────

class ExplorationPolicyNode(BTNode):
    """
    Dispatches the per-state mission directive.

    Mirrors the original _tick() state-dispatch block so the state
    machine logic is preserved exactly.  Always returns SUCCESS, meaning
    the priority selector stops here during normal (guard-free) operation.
    """

    def tick(self, node: Any) -> BTStatus:
        state = node.sm.state

        if state == "SURVEY":
            node._cmd_survey()

        elif state == "VERIFY_TAG":
            node._cmd_verify_tag()

        elif state == "PATH_VERIFY":
            node._cmd_path_verify()

        elif state == "CONVERGE":
            node._cmd_converge()

        elif state == "FINISH":
            node._cmd_finish()

        return BTStatus.SUCCESS


# ── Passive background leaf ───────────────────────────────────────────────────

class P2PSyncManagerNode(BTNode):
    """
    Passive background leaf – always returns SUCCESS.

    The P2P sync protocol is driven entirely by p2p_sync_node running as a
    separate ROS2 node.  This leaf exists as a guaranteed bottom-of-tree
    catch-all so the selector never propagates FAILURE to the root.
    """

    def tick(self, node: Any) -> BTStatus:
        return BTStatus.SUCCESS
