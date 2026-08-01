"""
state_machine.py
----------------
Pure-logic mission state machine.  No ROS2 imports.

States:
  BOOT         → waiting for all drones to be ready
  SURVEY       → distributed area scan
  VERIFY_TAG   → verifying mine candidates
  PATH_VERIFY  → flying the safe path to confirm it
  CONVERGE     → all drones sync final belief map
  FINISH       → submit and land

Transitions are driven by the mission_logic_node calling:
  sm.on_event(event, context)

where context is a dict with keys like:
  time_remaining, mine_count, confirmed_count, all_drones_ready,
  path_verified, all_converged, team_belief_count
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Callable, List
import time


# ── Arena constants (IARC Mission 10 spec: 300 ft × 80 ft) ───────────────────

ARENA_WIDTH    = 91.44   # metres (300 ft)
ARENA_HEIGHT   = 24.38   # metres (80 ft)
MISSION_DURATION = 420.0 # seconds (7 minutes)


# ── State definitions ─────────────────────────────────────────────────────────

STATES = [
    "BOOT",
    "SURVEY",
    "VERIFY_TAG",
    "PATH_VERIFY",
    "CONVERGE",
    "FINISH",
]

# Time thresholds (seconds remaining in mission) that trigger state transitions
# Adjusted for 7-minute (420 s) total mission time.
T_PATH_VERIFY = 90.0    # enter PATH_VERIFY when ≤ 90s left
T_CONVERGE    = 45.0    # enter CONVERGE when ≤ 45s left
T_FINISH      = 10.0    # enter FINISH when ≤ 10s left


@dataclass
class MissionContext:
    """Snapshot of mission state passed to transition checks."""
    time_remaining:    float = 420.0
    mine_count:        int   = 0       # candidates seen
    confirmed_count:   int   = 0       # confirmed mines
    all_drones_ready:  bool  = False
    path_verified:     bool  = False
    all_converged:     bool  = False
    team_belief_count: int   = 0       # total beliefs across team


class StateMachine:

    def __init__(self, drone_id: str, mission_duration: float = 420.0):
        self.drone_id         = drone_id
        self.mission_duration = mission_duration
        self.state            = "BOOT"
        self.state_entered_at = time.monotonic()
        self._history: List[tuple] = []   # (from, to, monotonic_time)
        self._on_transition: Optional[Callable] = None   # callback

    # ── Registration ──────────────────────────────────────────────────────────

    def on_transition(self, cb: Callable):
        """Register a callback: cb(from_state, to_state) called on every transition."""
        self._on_transition = cb

    # ── Transition ────────────────────────────────────────────────────────────

    def _go(self, new_state: str):
        if new_state == self.state:
            return
        old = self.state
        self.state = new_state
        self.state_entered_at = time.monotonic()
        self._history.append((old, new_state, self.state_entered_at))
        if self._on_transition:
            self._on_transition(old, new_state)

    # ── Main tick ─────────────────────────────────────────────────────────────

    def tick(self, ctx: MissionContext):
        """
        Called every ~1 s by mission_logic_node.
        Evaluates transition conditions and advances state if needed.
        """
        tr = ctx.time_remaining

        if self.state == "BOOT":
            if ctx.all_drones_ready:
                self._go("SURVEY")

        elif self.state == "SURVEY":
            # Time-based override: must start verifying before PATH_VERIFY window
            if tr <= T_PATH_VERIFY:
                self._go("PATH_VERIFY")
            elif ctx.mine_count > 0:
                self._go("VERIFY_TAG")

        elif self.state == "VERIFY_TAG":
            if tr <= T_PATH_VERIFY:
                self._go("PATH_VERIFY")
            # If all current candidates are resolved, go back to survey
            elif ctx.mine_count > 0 and ctx.confirmed_count == ctx.mine_count:
                self._go("SURVEY")

        elif self.state == "PATH_VERIFY":
            if tr <= T_CONVERGE:
                self._go("CONVERGE")
            elif ctx.path_verified:
                self._go("CONVERGE")

        elif self.state == "CONVERGE":
            if tr <= T_FINISH:
                self._go("FINISH")
            elif ctx.all_converged:
                self._go("FINISH")

        # FINISH is terminal

    # ── Helpers ───────────────────────────────────────────────────────────────

    def time_in_state(self) -> float:
        return time.monotonic() - self.state_entered_at

    def is_terminal(self) -> bool:
        return self.state == "FINISH"

    def summary(self) -> str:
        return (f"[{self.drone_id}] state={self.state} "
                f"({self.time_in_state():.1f}s in state)")
