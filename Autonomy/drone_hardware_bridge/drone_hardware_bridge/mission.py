"""Pure high-level command interpreter and execution bookkeeping."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import time
from typing import Callable, List, Optional, Sequence, Tuple

from .grid_planner import GridPlanner, Point, lawnmower_waypoints


@dataclass
class TaskContext:
    task_id: str
    task_type: str
    mine_id: str
    target: Point
    started: float
    arrived: Optional[float] = None


@dataclass
class MissionPlan:
    mode: str = "HOLD"
    goals: List[Point] = field(default_factory=list)
    path: List[Point] = field(default_factory=list)
    task: Optional[TaskContext] = None
    reason: str = "startup"


class CommandPlanner:
    SAFE_HOLD_COMMANDS = {"HOLD_POSITION", "STANDBY_FOR_TASK", "AWAIT_PATH_VERIFY"}

    def __init__(
        self,
        arena_width: float,
        arena_height: float,
        coverage_spacing_m: float,
        arena_map_aligned: bool,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.arena_width = arena_width
        self.arena_height = arena_height
        self.coverage_spacing_m = coverage_spacing_m
        self.arena_map_aligned = arena_map_aligned
        self.clock = clock
        self.plan = MissionPlan()

    def hold(self, current: Point, reason: str) -> MissionPlan:
        self.plan = MissionPlan(mode="HOLD", goals=[current], reason=reason)
        return self.plan

    def _validate_point(self, point: Point) -> None:
        if not all(math.isfinite(value) for value in point):
            raise ValueError("target is non-finite")
        if not (0.0 <= point[0] <= self.arena_width and 0.0 <= point[1] <= self.arena_height):
            raise ValueError("target is outside configured arena")

    def _route(
        self, current: Point, goals: Sequence[Point], planner: GridPlanner
    ) -> List[Point]:
        route: List[Point] = []
        start = current
        for goal in goals:
            self._validate_point(goal)
            segment = planner.plan(start, goal)
            if not segment:
                raise ValueError(f"no collision-free path to {goal}")
            route.extend(segment)
            start = goal
        return route

    def command(
        self,
        raw: str,
        current: Point,
        planner: GridPlanner,
    ) -> MissionPlan:
        try:
            data = json.loads(raw)
        except (json.JSONDecodeError, TypeError) as exc:
            return self.hold(current, f"invalid JSON: {exc}")
        if not isinstance(data, dict):
            return self.hold(current, "command is not an object")
        command = data.get("cmd")
        task_type = data.get("task_type")
        if command in self.SAFE_HOLD_COMMANDS:
            return self.hold(current, str(command))
        if command == "LAND_AND_SUBMIT":
            self.plan = MissionPlan(mode="LAND", reason="mission complete")
            return self.plan
        if not self.arena_map_aligned:
            return self.hold(current, "arena-to-map calibration not confirmed")
        try:
            task = None
            if command in ("SWEEP_SECTOR", "FILL_GAPS"):
                goals = lawnmower_waypoints(
                    float(data["x_min"]), float(data["x_max"]),
                    float(data["y_min"]), float(data["y_max"]),
                    self.coverage_spacing_m,
                )
                mode = "COVERAGE"
            elif command == "VERIFY_PATH":
                raw_waypoints = data.get("waypoints")
                if not isinstance(raw_waypoints, list) or not raw_waypoints:
                    raise ValueError("VERIFY_PATH requires waypoints")
                goals = [(float(point[0]), float(point[1])) for point in raw_waypoints]
                mode = "VERIFY_PATH"
                if data.get("task_id"):
                    task = TaskContext(
                        str(data["task_id"]), "VERIFY_PATH", "", goals[-1], self.clock()
                    )
            elif task_type == "VERIFY_TAG":
                goal = float(data["target_x"]), float(data["target_y"])
                goals = [goal]
                mode = "VERIFY_TAG"
                task = TaskContext(
                    str(data["task_id"]), "VERIFY_TAG", str(data.get("mine_id", "")),
                    goal, self.clock(),
                )
            else:
                return self.hold(current, f"unsupported command {command or task_type!r}")
            path = self._route(current, goals, planner)
            self.plan = MissionPlan(mode=mode, goals=list(goals), path=path, task=task)
            return self.plan
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            return self.hold(current, str(exc))

    def reached(self, current: Point, tolerance_m: float) -> bool:
        if not self.plan.path:
            return True
        target = self.plan.path[0]
        if math.hypot(target[0] - current[0], target[1] - current[1]) > tolerance_m:
            return False
        self.plan.path.pop(0)
        if not self.plan.path and self.plan.task is not None:
            self.plan.task.arrived = self.clock()
        return not self.plan.path

    def verification_result(
        self,
        raw: str,
        proximity_m: float,
    ) -> Optional[Tuple[str, str, str, float]]:
        task = self.plan.task
        if task is None or task.task_type != "VERIFY_TAG" or task.arrived is None:
            return None
        try:
            data = json.loads(raw)
            outcome = str(data["outcome"])
            confidence = float(data.get("confidence", 0.0))
            mine_id = str(data.get("mine_id", ""))
            if outcome not in {"confirmed", "rejected", "uncertain", "failed"}:
                return None
            if not 0.0 <= confidence <= 1.0:
                return None
            identity_matches = bool(task.mine_id and mine_id == task.mine_id)
            proximity_matches = False
            if "x" in data and "y" in data:
                proximity_matches = math.hypot(
                    float(data["x"]) - task.target[0], float(data["y"]) - task.target[1]
                ) <= proximity_m
            if not identity_matches and not proximity_matches:
                return None
            return task.task_id, mine_id or task.mine_id, outcome, confidence
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return None
