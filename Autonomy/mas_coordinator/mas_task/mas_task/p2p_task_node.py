"""
p2p_task_node.py
----------------
Fully P2P task auction node.  Every drone runs an identical copy.

Responsibilities:
  - Listen for /team/task_announce  → decide whether to bid
  - Publish /team/task_claim        → bid with computed cost
  - Listen for all claims           → feed into AuctionManager
  - On tick: resolve expired auctions → if winner, send task_cmd to explorer
  - Listen for /team/task_result    → update local belief store via sync node

Parameters:
  drone_id          : str
  tick_rate         : float   Hz for auction resolution tick (default 5.0)
  announce_cooldown : float   s between re-announcing the same candidate (default 10.0)
"""

import rclpy
from rclpy.node import Node
import time
from typing import Dict, List, Set, Optional, Tuple

from mas_interfaces.msg import (
    TaskAnnounce, TaskClaim, TaskResult, MineBelief
)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String   # task_cmd to explorer: JSON string

from .auction_manager import AuctionManager, TaskAnnounceData, ClaimData


class P2PTaskNode(Node):

    def __init__(self):
        super().__init__("p2p_task_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id",          "d1")
        self.declare_parameter("tick_rate",          5.0)
        self.declare_parameter("announce_cooldown", 10.0)
        self.declare_parameter("pose_timeout_s",      3.0)
        self.declare_parameter("task_timeout_s",     30.0)
        self.declare_parameter("retry_delay_s",       5.0)
        self.declare_parameter("max_retries",           3)

        self.drone_id         = self.get_parameter("drone_id").value
        tick_rate             = self.get_parameter("tick_rate").value
        self.announce_cooldown= self.get_parameter("announce_cooldown").value
        self.pose_timeout_s   = self.get_parameter("pose_timeout_s").value
        self.task_timeout_s   = self.get_parameter("task_timeout_s").value
        self.retry_delay_s    = self.get_parameter("retry_delay_s").value
        self.max_retries      = self.get_parameter("max_retries").value

        # ── State ─────────────────────────────────────────────────────────────
        self.my_x      = 0.0
        self.my_y      = 0.0
        self.my_state  = "BOOT"          # updated by mission_logic_node
        self._pose_last_seen: Optional[float] = None
        self.busy      = False           # True while executing a won task
        self.current_task_id: Optional[str] = None
        self._current_task_started: Optional[float] = None

        # Tracks task_ids we have already announced (to avoid spam)
        self._announced: Dict[str, float] = {}  # task_id -> monotonic time

        # Mine IDs that are already confirmed/rejected — skip re-announcing
        self._resolved_mines: Set[str] = set()

        # Tasks awaiting retry (no bidders): (retry_at, TaskAnnounceData)
        self._pending_retry: List[Tuple[float, "TaskAnnounceData"]] = []
        self._retry_counts: Dict[str, int] = {}
        self._task_roots: Dict[str, str] = {}
        self._retry_scheduled: Set[str] = set()

        self.auction_mgr = AuctionManager(self.drone_id)
        self._seq = 0   # for generating unique task_ids

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_announce = self.create_publisher(TaskAnnounce, "/team/task_announce", 10)
        self.pub_claim    = self.create_publisher(TaskClaim,    "/team/task_claim",    10)
        self.pub_result   = self.create_publisher(TaskResult,   "/team/task_result",   10)

        # Send winning task to the local explorer node
        self.pub_task_cmd = self.create_publisher(
            String, f"/{self.drone_id}/task_cmd", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            TaskAnnounce, "/team/task_announce",
            self._on_task_announce, 20)

        self.create_subscription(
            TaskClaim, "/team/task_claim",
            self._on_task_claim, 20)

        self.create_subscription(
            TaskResult, "/team/task_result",
            self._on_task_result, 10)

        # Local pose (to compute bid cost)
        self.create_subscription(
            PoseStamped, f"/{self.drone_id}/pose",
            self._on_local_pose, 10)

        self.create_subscription(
            String, f"/{self.drone_id}/mission_state",
            self._on_mission_state, 10)

        # Mine candidates from local detector / explorer
        self.create_subscription(
            MineBelief, f"/{self.drone_id}/mine_candidates",
            self._on_mine_candidate, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / tick_rate, self._tick)

        self.get_logger().info(f"[{self.drone_id}] p2p_task_node ready")

    # ── Pose ──────────────────────────────────────────────────────────────────

    def _on_local_pose(self, msg: PoseStamped):
        self.my_x = msg.pose.position.x
        self.my_y = msg.pose.position.y
        self._pose_last_seen = time.monotonic()

    def _on_mission_state(self, msg: String):
        self.my_state = msg.data

    def _pose_is_fresh(self) -> bool:
        return (self._pose_last_seen is not None and
                time.monotonic() - self._pose_last_seen <= self.pose_timeout_s)

    # ── Mine candidate → announce auction ─────────────────────────────────────

    def _on_mine_candidate(self, msg: MineBelief):
        """
        When explorer detects a new mine candidate, announce a VERIFY_TAG task.
        Cooldown prevents spamming; resolved mines are never re-announced.
        """
        if msg.mine_id in self._resolved_mines:
            return   # already confirmed or rejected — skip

        candidate_key = msg.mine_id
        now = time.monotonic()

        last = self._announced.get(candidate_key, 0.0)
        if now - last < self.announce_cooldown:
            return   # already announced recently

        self._announced[candidate_key] = now
        self._seq += 1
        task_id = f"verify_{msg.mine_id}__{self.drone_id}_{self._seq}"
        self.announce_task(
            task_id=task_id,
            task_type="VERIFY_TAG",
            target_x=msg.x,
            target_y=msg.y,
            priority=msg.confidence,
            claim_window_s=3.0,
            mine_id=msg.mine_id,
        )

    # ── Announce ──────────────────────────────────────────────────────────────

    def announce_task(self, task_id: str, task_type: str,
                      target_x: float, target_y: float,
                      priority: float = 0.5,
                      claim_window_s: float = 2.0,
                      mine_id: str = ""):
        """
        Publish a task announcement.  Can be called by mission_logic_node too
        (e.g. for BECOME_PATH_VERIFIER, RESCAN tasks).
        """
        msg = TaskAnnounce()
        msg.task_id        = task_id
        msg.task_type      = task_type
        msg.announcer_id   = self.drone_id
        msg.mine_id        = mine_id
        msg.target_x       = target_x
        msg.target_y       = target_y
        msg.priority       = priority
        msg.claim_window_s = claim_window_s
        msg.stamp          = self.get_clock().now().to_msg()
        data = TaskAnnounceData(
            task_id=task_id,
            task_type=task_type,
            announcer_id=self.drone_id,
            target_x=target_x,
            target_y=target_y,
            priority=priority,
            claim_window_s=claim_window_s,
            mine_id=mine_id,
        )
        # Register and bid locally.  The looped-back DDS sample is then a
        # duplicate and cannot produce a second bid.
        if self.auction_mgr.on_announce(data):
            self._bid_if_eligible(data)
        self.pub_announce.publish(msg)
        self.get_logger().info(
            f"[{self.drone_id}] Announced {task_type} task {task_id}")

    # ── Incoming announce → decide to bid ─────────────────────────────────────

    def _on_task_announce(self, msg: TaskAnnounce):
        data = TaskAnnounceData(
            task_id=msg.task_id,
            task_type=msg.task_type,
            announcer_id=msg.announcer_id,
            mine_id=msg.mine_id,
            target_x=msg.target_x,
            target_y=msg.target_y,
            priority=msg.priority,
            claim_window_s=msg.claim_window_s,
        )
        if not self.auction_mgr.on_announce(data):
            return
        self._bid_if_eligible(data)

    def _bid_if_eligible(self, data: TaskAnnounceData):
        """Publish one bid for a newly registered auction, if safe."""
        if not self._pose_is_fresh():
            return

        cost = self.auction_mgr.compute_cost(
            data, self.my_x, self.my_y, self.my_state, self.busy)

        if cost is not None:
            claim = TaskClaim()
            claim.task_id   = data.task_id
            claim.bidder_id = self.drone_id
            claim.cost      = cost
            claim.stamp     = self.get_clock().now().to_msg()
            self.pub_claim.publish(claim)

    # ── Incoming claim → feed into auction manager ─────────────────────────────

    def _on_task_claim(self, msg: TaskClaim):
        self.auction_mgr.on_claim(ClaimData(
            task_id=msg.task_id,
            bidder_id=msg.bidder_id,
            cost=msg.cost,
        ))

    # ── Tick: resolve expired auctions ────────────────────────────────────────

    def _tick(self):
        self._expire_current_task()
        self.auction_mgr.tick()
        won_tasks = self.auction_mgr.pop_won_tasks()
        abandoned = self.auction_mgr.pop_abandoned_tasks()

        for task in won_tasks:
            self._execute_won_task(task)

        # Schedule abandoned tasks for retry in 5 s
        now = time.monotonic()
        for task in abandoned:
            # Only the original announcer owns retries.  Retrying from every
            # peer used to create storms of duplicate, permanently closed IDs.
            if task.announcer_id != self.drone_id:
                continue
            root_id = self._task_roots.get(task.task_id, task.task_id)
            retry_count = self._retry_counts.get(root_id, 0)
            if retry_count >= self.max_retries:
                self.get_logger().error(
                    f"[{self.drone_id}] Task {root_id} abandoned after "
                    f"{retry_count} retries")
                continue
            retry_at = now + self.retry_delay_s
            self._pending_retry.append((retry_at, task))
            self.get_logger().warn(
                f"[{self.drone_id}] Task {task.task_id} had no bidders — "
                f"retrying in {self.retry_delay_s:.1f}s")

        # Fire any retries that are due
        still_pending = []
        for retry_at, task in self._pending_retry:
            if now >= retry_at:
                root_id = self._task_roots.get(task.task_id, task.task_id)
                retry_count = self._retry_counts.get(root_id, 0) + 1
                self._retry_counts[root_id] = retry_count
                retry_task_id = f"{root_id}__retry_{retry_count}"
                self._task_roots[retry_task_id] = root_id
                self._retry_scheduled.discard(task.task_id)
                self.get_logger().info(
                    f"[{self.drone_id}] Retrying task {root_id} as "
                    f"{retry_task_id}")
                self.announce_task(
                    task_id=retry_task_id,
                    task_type=task.task_type,
                    target_x=task.target_x,
                    target_y=task.target_y,
                    priority=task.priority,
                    claim_window_s=task.claim_window_s,
                    mine_id=task.mine_id,
                )
            else:
                still_pending.append((retry_at, task))
        self._pending_retry = still_pending

    def _schedule_failed_retry(self, task: TaskAnnounceData):
        """Schedule one announcer-owned retry after executor failure."""
        if (task.announcer_id != self.drone_id or
                task.task_id in self._retry_scheduled):
            return
        root_id = self._task_roots.get(task.task_id, task.task_id)
        retry_count = self._retry_counts.get(root_id, 0)
        if retry_count >= self.max_retries:
            self.get_logger().error(
                f"[{self.drone_id}] Task {root_id} failed after "
                f"{retry_count} retries")
            return
        self._retry_scheduled.add(task.task_id)
        self._pending_retry.append(
            (time.monotonic() + self.retry_delay_s, task))
        self.get_logger().warn(
            f"[{self.drone_id}] Failed task {task.task_id} will retry in "
            f"{self.retry_delay_s:.1f}s")

    def _execute_won_task(self, task: TaskAnnounceData):
        """
        This drone won the auction.  Send task_cmd to the local explorer.
        Uses a simple JSON string so explorer_node can parse without custom msgs.
        """
        if task.task_type in ("BECOME_PATH_VERIFIER", "BECOME_VERIFIER"):
            # Role assignment is instantaneous; the result broadcasts the
            # elected winner to every mission node.
            self.report_result(
                task.task_id, "", "assigned", 1.0, clear_current=False)
            return

        import json
        self.busy = True
        self.current_task_id = task.task_id
        self._current_task_started = time.monotonic()

        cmd = json.dumps({
            "task_id":   task.task_id,
            "task_type": task.task_type,
            "mine_id":   task.mine_id,
            "target_x":  task.target_x,
            "target_y":  task.target_y,
            "priority":  task.priority,
        })

        msg = String()
        msg.data = cmd
        self.pub_task_cmd.publish(msg)

        self.get_logger().info(
            f"[{self.drone_id}] Won task {task.task_id} ({task.task_type}) "
            f"→ dispatched to explorer")

    # ── Task result ───────────────────────────────────────────────────────────

    def _on_task_result(self, msg: TaskResult):
        """
        When a result arrives (could be ours or another drone's),
        mark ourselves as no longer busy if it was our task.
        """
        if msg.outcome in ("confirmed", "rejected") and msg.mine_id:
            self._resolved_mines.add(msg.mine_id)

        if msg.outcome in ("failed", "uncertain"):
            task = self.auction_mgr.get_announce(msg.task_id)
            if task is not None:
                self._schedule_failed_retry(task)

        if msg.executor_id == self.drone_id and msg.task_id == self.current_task_id:
            self.busy = False
            self.current_task_id = None
            self._current_task_started = None
            self.get_logger().info(
                f"[{self.drone_id}] Task {msg.task_id} completed: {msg.outcome}")

    # ── Called by explorer when verification is done ──────────────────────────

    def report_result(self, task_id: str, mine_id: str,
                      outcome: str, confidence: float,
                      clear_current: bool = True):
        """
        Explorer calls this after finishing a VERIFY_TAG task.
        Publishes result to the team.
        """
        msg = TaskResult()
        msg.task_id     = task_id
        msg.executor_id = self.drone_id
        msg.outcome     = outcome
        msg.mine_id     = mine_id
        msg.confidence  = confidence
        msg.stamp       = self.get_clock().now().to_msg()
        self.pub_result.publish(msg)
        if clear_current:
            self.busy = False
            self.current_task_id = None
            self._current_task_started = None

    def _expire_current_task(self):
        """Release a task whose executor never reports completion."""
        if (not self.busy or self.current_task_id is None or
                self._current_task_started is None):
            return
        age = time.monotonic() - self._current_task_started
        if age <= self.task_timeout_s:
            return
        task_id = self.current_task_id
        self.get_logger().error(
            f"[{self.drone_id}] Task {task_id} timed out after {age:.1f}s")
        self.report_result(task_id, "", "failed", 0.0)

    # ── State update (called by mission_logic_node) ───────────────────────────

# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = P2PTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
