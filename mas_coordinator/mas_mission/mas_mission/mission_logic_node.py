"""
mission_logic_node.py
---------------------
Top-level mission coordinator.  Owns the StateMachine and orchestrates
the sync node + task node based on current state.

Responsibilities:
  - Track mission timer
  - Build MissionContext from team beliefs and drone states
  - Drive state machine tick
  - Issue high-level directives per state:
      SURVEY       → tell explorer to sweep the full arena
      VERIFY_TAG   → announce VERIFY_TAG tasks for unconfirmed mines
      PATH_VERIFY  → auction BECOME_PATH_VERIFIER role; verifier flies the
                     corridor, explorer fills gaps
      CONVERGE     → broadcast final occupancy grid, rescan low-confidence mines
      FINISH       → trigger land + submit

Parameters:
  drone_id          : str
  mission_duration  : float   total mission time in seconds (default 420)
  num_drones        : int     expected team size (default 4)
  arena_width       : float   arena width in metres    (default 91.44 — 300 ft)
  arena_height      : float   arena height in metres   (default 24.38 — 80 ft)
"""

import rclpy
from rclpy.node import Node
import time
import json
import math
from typing import Dict, Optional, List

from mas_interfaces.msg import (
    PoseBeacon, TaskAnnounce, TaskResult, MineBelief, MineDelta
)
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from .state_machine import StateMachine, MissionContext, ARENA_WIDTH, ARENA_HEIGHT, MISSION_DURATION
from .bt_runner import (
    PrioritySelector,
    CollisionGuardNode,
    GeofenceGuardNode,
    FailureMonitorNode,
    TaskExecutorNode,
    ExplorationPolicyNode,
    P2PSyncManagerNode,
)


# ── Main node ─────────────────────────────────────────────────────────────────

class MissionLogicNode(Node):

    def __init__(self):
        super().__init__("mission_logic_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id",         "d1")
        self.declare_parameter("mission_duration", MISSION_DURATION)
        self.declare_parameter("num_drones",        4)
        self.declare_parameter("arena_width",       ARENA_WIDTH)
        self.declare_parameter("arena_height",      ARENA_HEIGHT)

        self.drone_id        = self.get_parameter("drone_id").value
        mission_duration     = self.get_parameter("mission_duration").value
        self.num_drones      = self.get_parameter("num_drones").value
        arena_w              = self.get_parameter("arena_width").value
        arena_h              = self.get_parameter("arena_height").value

        # Arena bounds (full arena for all drones; Kevin's A* handles exploration)
        self.arena_w = arena_w
        self.arena_h = arena_h

        # ── Mission state ─────────────────────────────────────────────────────
        self.sm = StateMachine(self.drone_id, mission_duration)
        self.sm.on_transition(self._on_state_transition)

        self.mission_start: Optional[float] = None   # set on BOOT→SURVEY transition
        self.mission_duration = mission_duration

        # All drones share identical full-arena bounds; Kevin's explorer does path planning
        self.sector = {
            "x_min": 0.0,     "x_max": arena_w,
            "y_min": 0.0,     "y_max": arena_h,
            "cx":    arena_w / 2,
            "cy":    arena_h / 2,
        }

        # Own pose (updated from PoseBeacon; needed by collision/geofence guards)
        self.own_x: float = 0.0
        self.own_y: float = 0.0
        self.own_pose_last_seen: float = time.monotonic()  # FailureMonitor baseline

        # Team awareness
        self.team_states: Dict[str, str] = {}        # drone_id → state
        self.team_last_seen: Dict[str, float] = {}   # drone_id → monotonic
        self.team_poses: Dict[str, tuple] = {}       # drone_id → (x, y)

        # Pending task command from p2p_task_node (consumed by TaskExecutorNode)
        self.pending_task_cmd: Optional[str] = None

        # Mine beliefs (mirrored from sync node via /team/mine_delta)
        self.mine_beliefs: Dict[str, dict] = {}      # mine_id → dict
        self.path_verified = False

        # PATH_VERIFY task tracking
        self._path_task_announced = False
        self._path_task_id: Optional[str] = None
        self._path_verifier_role: Optional[str] = None   # "verifier" | "explorer"

        # CONVERGE task tracking
        self._converge_verifier_announced = False
        self._converge_verifier_role: Optional[str] = None  # "verifier" | "explorer"
        self._converge_rescans_announced = False

        # Registry of task_id → task_type for tasks this node announces
        # (TaskResult has no task_type field, so we look it up here)
        self._task_registry: Dict[str, str] = {}

        # Occupancy grid (built during CONVERGE)
        self._grid_published = False

        # ── Publishers ────────────────────────────────────────────────────────
        # High-level command to explorer_node
        self.pub_mission_cmd = self.create_publisher(
            String, f"/{self.drone_id}/mission_cmd", 10)

        # Final occupancy grid (for scoring)
        self.pub_grid = self.create_publisher(
            OccupancyGrid, f"/{self.drone_id}/safe_path_grid", 10)

        # Task announcements (path verify)
        self.pub_announce = self.create_publisher(
            TaskAnnounce, "/team/task_announce", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseBeacon, "/team/pose_beacon",
            self._on_pose_beacon, 20)

        self.create_subscription(
            MineDelta, "/team/mine_delta",
            self._on_mine_delta, 20)

        self.create_subscription(
            TaskResult, "/team/task_result",
            self._on_task_result, 10)

        # Task commands won by the p2p task node (buffered for TaskExecutorNode)
        self.create_subscription(
            String, f"/{self.drone_id}/task_cmd",
            self._on_task_cmd, 10)

        # ── Behaviour Tree ────────────────────────────────────────────────────
        self.bt_tree = PrioritySelector([
            CollisionGuardNode(),
            GeofenceGuardNode(),
            FailureMonitorNode(),
            TaskExecutorNode(),
            ExplorationPolicyNode(),
            P2PSyncManagerNode(),
        ])

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"[{self.drone_id}] mission_logic_node ready | "
            f"arena={arena_w:.2f}x{arena_h:.2f}m | "
            f"mission_duration={mission_duration:.0f}s")

    # ── Team awareness ────────────────────────────────────────────────────────

    def _on_pose_beacon(self, msg: PoseBeacon):
        if msg.drone_id == self.drone_id:
            # Track own pose for collision/geofence/failure guards
            self.own_x = msg.x
            self.own_y = msg.y
            self.own_pose_last_seen = time.monotonic()
            return
        now = time.monotonic()
        self.team_states[msg.drone_id] = msg.state
        self.team_last_seen[msg.drone_id] = now
        self.team_poses[msg.drone_id] = (msg.x, msg.y)

    # ── Belief mirroring ──────────────────────────────────────────────────────

    def _on_mine_delta(self, msg: MineDelta):
        for b in msg.beliefs:
            existing = self.mine_beliefs.get(b.mine_id)
            if existing is None or b.seq > existing.get("seq", -1):
                self.mine_beliefs[b.mine_id] = {
                    "mine_id":    b.mine_id,
                    "x": b.x, "y": b.y,
                    "confidence": b.confidence,
                    "status":     b.status,
                    "seq":        b.seq,
                }

    def _on_task_result(self, msg: TaskResult):
        # ── Path verification complete ─────────────────────────────────────────
        if msg.task_id == self._path_task_id and msg.outcome == "confirmed":
            self.path_verified = True
            self.get_logger().info(
                f"[{self.drone_id}] Path verification confirmed!")

        # ── Role assignment from tasks we announced ────────────────────────────
        task_type = self._task_registry.get(msg.task_id)

        if task_type == "BECOME_PATH_VERIFIER":
            if msg.executor_id == self.drone_id:
                self._path_verifier_role = "verifier"
                self.get_logger().info(
                    f"[{self.drone_id}] Won BECOME_PATH_VERIFIER → role=verifier")
            else:
                self._path_verifier_role = "explorer"
                self.get_logger().info(
                    f"[{self.drone_id}] Lost BECOME_PATH_VERIFIER to "
                    f"{msg.executor_id} → role=explorer")

        elif task_type == "BECOME_VERIFIER":
            if msg.executor_id == self.drone_id:
                self._converge_verifier_role = "verifier"
                self.get_logger().info(
                    f"[{self.drone_id}] Won BECOME_VERIFIER → role=verifier")
            else:
                self._converge_verifier_role = "explorer"
                self.get_logger().info(
                    f"[{self.drone_id}] Lost BECOME_VERIFIER to "
                    f"{msg.executor_id} → role=explorer")

        # ── Mine belief update from any VERIFY_TAG result ─────────────────────
        if (task_type == "VERIFY_TAG"
                and msg.outcome in ("confirmed", "rejected")
                and msg.mine_id in self.mine_beliefs):
            self.mine_beliefs[msg.mine_id]["status"]     = msg.outcome
            self.mine_beliefs[msg.mine_id]["confidence"] = msg.confidence
            self.get_logger().info(
                f"[{self.drone_id}] Mine {msg.mine_id} updated: "
                f"status={msg.outcome} confidence={msg.confidence:.2f}")

    def _on_task_cmd(self, msg: String):
        """Buffer the latest task_cmd from p2p_task_node for TaskExecutorNode."""
        self.pending_task_cmd = msg.data

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _time_remaining(self) -> float:
        if self.mission_start is None:
            return self.mission_duration
        elapsed = time.monotonic() - self.mission_start
        return max(0.0, self.mission_duration - elapsed)

    def _all_drones_ready(self) -> bool:
        """True if we've heard from all expected drones recently (< 3s ago)."""
        now = time.monotonic()
        active = {did for did, t in self.team_last_seen.items()
                  if now - t < 3.0}
        return len(active) >= self.num_drones - 1   # -1 because we don't count ourselves

    def _all_converged(self) -> bool:
        """All drones are in CONVERGE or FINISH state."""
        if not self.team_states:
            return False
        return all(s in ("CONVERGE", "FINISH")
                   for s in self.team_states.values())

    def _mine_context(self):
        all_beliefs = list(self.mine_beliefs.values())
        candidates  = [b for b in all_beliefs if b["status"] == "candidate"]
        confirmed   = [b for b in all_beliefs if b["status"] == "confirmed"]
        return len(all_beliefs), len(confirmed)

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        # Prune drones not seen for > 5 s (dropout detection)
        now = time.monotonic()
        dropped = [did for did, t in self.team_last_seen.items()
                   if now - t > 5.0]
        for did in dropped:
            self.team_last_seen.pop(did, None)
            self.team_states.pop(did, None)
            self.team_poses.pop(did, None)
            self.get_logger().warn(
                f"[{self.drone_id}] Drone {did} dropped out (no beacon for 5s)")

        mine_count, confirmed_count = self._mine_context()

        ctx = MissionContext(
            time_remaining=self._time_remaining(),
            mine_count=mine_count,
            confirmed_count=confirmed_count,
            all_drones_ready=self._all_drones_ready(),
            path_verified=self.path_verified,
            all_converged=self._all_converged(),
            team_belief_count=len(self.mine_beliefs),
        )

        self.sm.tick(ctx)

        # Run the BT priority selector.  Guards fire first; exploration
        # policy runs only when no guard has claimed the tick.
        self.bt_tree.tick(self)

    # ── State directives ──────────────────────────────────────────────────────

    def _cmd_survey(self):
        """Tell explorer to sweep our assigned sector."""
        cmd = json.dumps({
            "cmd":   "SWEEP_SECTOR",
            "x_min": self.sector["x_min"],
            "x_max": self.sector["x_max"],
            "y_min": self.sector["y_min"],
            "y_max": self.sector["y_max"],
        })
        self._publish_cmd(cmd)

    def _cmd_verify_tag(self):
        """
        For each unconfirmed candidate, the task node handles announcing
        VERIFY_TAG tasks.  Mission node just ensures explorer stays in
        'ready for tasking' mode.
        """
        cmd = json.dumps({"cmd": "STANDBY_FOR_TASK"})
        self._publish_cmd(cmd)

    def _cmd_path_verify(self):
        """
        Auction the path-verifier role once, then issue a role-specific command.

        BECOME_PATH_VERIFIER winner → VERIFY_PATH with sector corner waypoints.
        Loser                       → FILL_GAPS covering the assigned sector.
        Role not yet assigned       → AWAIT_PATH_VERIFY (waiting for auction).
        """
        if self.sm.state != "PATH_VERIFY":
            return

        if not self._path_task_announced:
            self._path_task_announced = True
            self._path_task_id = f"become_pv_{self.drone_id}_0"
            self._announce_task(
                self._path_task_id, "BECOME_PATH_VERIFIER",
                self.sector["cx"], self.sector["cy"],
                priority=1.0, claim_window_s=3.0,
            )

        if self._path_verifier_role == "verifier":
            corners = [
                [self.sector["x_min"], self.sector["y_min"]],
                [self.sector["x_max"], self.sector["y_min"]],
                [self.sector["x_max"], self.sector["y_max"]],
                [self.sector["x_min"], self.sector["y_max"]],
            ]
            cmd = json.dumps({"cmd": "VERIFY_PATH", "waypoints": corners})

        elif self._path_verifier_role == "explorer":
            cmd = json.dumps({
                "cmd":   "FILL_GAPS",
                "x_min": self.sector["x_min"],
                "x_max": self.sector["x_max"],
                "y_min": self.sector["y_min"],
                "y_max": self.sector["y_max"],
            })

        else:
            cmd = json.dumps({"cmd": "AWAIT_PATH_VERIFY"})

        self._publish_cmd(cmd)

    def _cmd_converge(self):
        """
        Converge phase:
          1. Publish the final occupancy grid once.
          2. Auction the BECOME_VERIFIER role once.
          3. Winner: announce VERIFY_TAG rescan tasks for low-confidence candidates.
          4. All drones: hold position while rescans complete.
        """
        if not self._grid_published:
            self._publish_occupancy_grid()
            self._grid_published = True

        if not self._converge_verifier_announced:
            self._converge_verifier_announced = True
            task_id = f"become_verifier_{self.drone_id}_0"
            self._announce_task(
                task_id, "BECOME_VERIFIER",
                self.sector["cx"], self.sector["cy"],
                priority=1.0, claim_window_s=3.0,
            )

        if self._converge_verifier_role == "verifier" and not self._converge_rescans_announced:
            self._converge_rescans_announced = True
            count = 0
            for mine_id, belief in self.mine_beliefs.items():
                if belief["status"] == "candidate" and belief["confidence"] < 0.7:
                    self._announce_task(
                        f"rescan_{mine_id}_{self.drone_id}", "VERIFY_TAG",
                        belief["x"], belief["y"],
                        priority=0.9, claim_window_s=2.0,
                    )
                    count += 1
            self.get_logger().info(
                f"[{self.drone_id}] Announced {count} rescan task(s) in CONVERGE")

        cmd = json.dumps({"cmd": "HOLD_POSITION"})
        self._publish_cmd(cmd)

    def _cmd_finish(self):
        cmd = json.dumps({"cmd": "LAND_AND_SUBMIT"})
        self._publish_cmd(cmd)
        self.get_logger().info(f"[{self.drone_id}] FINISH — landing")

    # ── Occupancy grid builder ─────────────────────────────────────────────────

    def _publish_occupancy_grid(self):
        """
        Build a simple occupancy grid:
          -1 = unknown
           0 = safe (no mine)
         100 = mine confirmed
        """
        resolution = 0.5   # metres per cell
        width_cells  = int(self.arena_w / resolution)
        height_cells = int(self.arena_h / resolution)

        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width      = width_cells
        grid.info.height     = height_cells
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0

        # Default: all safe
        data = [0] * (width_cells * height_cells)

        # Mark confirmed mines as occupied
        for b in self.mine_beliefs.values():
            if b["status"] == "confirmed":
                cx = int(b["x"] / resolution)
                cy = int(b["y"] / resolution)
                if 0 <= cx < width_cells and 0 <= cy < height_cells:
                    data[cy * width_cells + cx] = 100

        grid.data = data
        self.pub_grid.publish(grid)
        self.get_logger().info(
            f"[{self.drone_id}] Published occupancy grid "
            f"({width_cells}x{height_cells} cells)")

    # ── Utils ─────────────────────────────────────────────────────────────────

    def _announce_task(self, task_id: str, task_type: str,
                       target_x: float, target_y: float,
                       priority: float, claim_window_s: float) -> None:
        """Publish a TaskAnnounce and register task_id → task_type locally."""
        msg = TaskAnnounce()
        msg.task_id        = task_id
        msg.task_type      = task_type
        msg.announcer_id   = self.drone_id
        msg.target_x       = target_x
        msg.target_y       = target_y
        msg.priority       = priority
        msg.claim_window_s = claim_window_s
        msg.stamp          = self.get_clock().now().to_msg()
        self.pub_announce.publish(msg)
        self._task_registry[task_id] = task_type
        self.get_logger().info(
            f"[{self.drone_id}] Announced {task_type} task {task_id}")

    def _publish_cmd(self, cmd_json: str):
        msg = String()
        msg.data = cmd_json
        self.pub_mission_cmd.publish(msg)

    def _on_state_transition(self, from_state: str, to_state: str):
        self.get_logger().info(
            f"[{self.drone_id}] State: {from_state} → {to_state} "
            f"(t_remaining={self._time_remaining():.0f}s)")

        if from_state == "BOOT" and to_state == "SURVEY":
            self.mission_start = time.monotonic()
            self.get_logger().info(
                f"[{self.drone_id}] Mission timer started")

        if to_state == "SURVEY":
            # Reset PATH_VERIFY state so re-entry works correctly
            self._path_task_announced = False
            self._path_task_id        = None
            self._path_verifier_role  = None

        elif to_state == "PATH_VERIFY":
            # Reset CONVERGE state ahead of possible entry
            self._converge_verifier_announced = False
            self._converge_verifier_role      = None
            self._converge_rescans_announced  = False


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
