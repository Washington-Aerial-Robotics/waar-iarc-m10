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
import math
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

        self.drone_id         = self.get_parameter("drone_id").value
        tick_rate             = self.get_parameter("tick_rate").value
        self.announce_cooldown= self.get_parameter("announce_cooldown").value

        # ── State ─────────────────────────────────────────────────────────────
        self.my_x      = 0.0
        self.my_y      = 0.0
        self.my_state  = "BOOT"          # updated by mission_logic_node
        self.busy      = False           # True while executing a won task
        self.current_task_id: Optional[str] = None

        # Tracks task_ids we have already announced (to avoid spam)
        self._announced: Dict[str, float] = {}  # task_id -> monotonic time

        # Mine IDs that are already confirmed/rejected — skip re-announcing
        self._resolved_mines: Set[str] = set()

        # Tasks awaiting retry (no bidders): (retry_at, TaskAnnounceData)
        self._pending_retry: List[Tuple[float, "TaskAnnounceData"]] = []

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

    # ── Mine candidate → announce auction ─────────────────────────────────────

    def _on_mine_candidate(self, msg: MineBelief):
        """
        When explorer detects a new mine candidate, announce a VERIFY_TAG task.
        Cooldown prevents spamming; resolved mines are never re-announced.
        """
        if msg.mine_id in self._resolved_mines:
            return   # already confirmed or rejected — skip

        task_id = f"verify_{msg.mine_id}"
        now = time.monotonic()

        last = self._announced.get(task_id, 0.0)
        if now - last < self.announce_cooldown:
            return   # already announced recently

        self._announced[task_id] = now
        self.announce_task(
            task_id=task_id,
            task_type="VERIFY_TAG",
            target_x=msg.x,
            target_y=msg.y,
            priority=msg.confidence,
            claim_window_s=3.0,
        )

    # ── Announce ──────────────────────────────────────────────────────────────

    def announce_task(self, task_id: str, task_type: str,
                      target_x: float, target_y: float,
                      priority: float = 0.5,
                      claim_window_s: float = 2.0):
        """
        Publish a task announcement.  Can be called by mission_logic_node too
        (e.g. for BECOME_PATH_VERIFIER, RESCAN tasks).
        """
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

        # Also register in our own auction manager
        data = TaskAnnounceData(
            task_id=task_id,
            task_type=task_type,
            announcer_id=self.drone_id,
            target_x=target_x,
            target_y=target_y,
            priority=priority,
            claim_window_s=claim_window_s,
        )
        self.auction_mgr.on_announce(data)
        self.get_logger().info(
            f"[{self.drone_id}] Announced {task_type} task {task_id}")

    # ── Incoming announce → decide to bid ─────────────────────────────────────

    def _on_task_announce(self, msg: TaskAnnounce):
        data = TaskAnnounceData(
            task_id=msg.task_id,
            task_type=msg.task_type,
            announcer_id=msg.announcer_id,
            target_x=msg.target_x,
            target_y=msg.target_y,
            priority=msg.priority,
            claim_window_s=msg.claim_window_s,
        )
        self.auction_mgr.on_announce(data)

        # Compute cost and bid if willing
        cost = self.auction_mgr.compute_cost(
            data, self.my_x, self.my_y, self.my_state, self.busy)

        if cost is not None:
            claim = TaskClaim()
            claim.task_id   = msg.task_id
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
        resolved = self.auction_mgr.tick()
        won_tasks = self.auction_mgr.pop_won_tasks()
        abandoned = self.auction_mgr.pop_abandoned_tasks()

        for task in won_tasks:
            self._execute_won_task(task)

        # Schedule abandoned tasks for retry in 5 s
        now = time.monotonic()
        for task in abandoned:
            retry_at = now + 5.0
            self._pending_retry.append((retry_at, task))
            self.get_logger().warn(
                f"[{self.drone_id}] Task {task.task_id} had no bidders — "
                f"retrying in 5s")

        # Fire any retries that are due
        still_pending = []
        for retry_at, task in self._pending_retry:
            if now >= retry_at:
                self.get_logger().info(
                    f"[{self.drone_id}] Retrying task {task.task_id}")
                self.announce_task(
                    task_id=task.task_id,
                    task_type=task.task_type,
                    target_x=task.target_x,
                    target_y=task.target_y,
                    priority=task.priority,
                    claim_window_s=task.claim_window_s,
                )
            else:
                still_pending.append((retry_at, task))
        self._pending_retry = still_pending

    def _execute_won_task(self, task: TaskAnnounceData):
        """
        This drone won the auction.  Send task_cmd to the local explorer.
        Uses a simple JSON string so explorer_node can parse without custom msgs.
        """
        import json
        self.busy = True
        self.current_task_id = task.task_id

        cmd = json.dumps({
            "task_id":   task.task_id,
            "task_type": task.task_type,
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

        if msg.executor_id == self.drone_id and msg.task_id == self.current_task_id:
            self.busy = False
            self.current_task_id = None
            self.get_logger().info(
                f"[{self.drone_id}] Task {msg.task_id} completed: {msg.outcome}")

    # ── Called by explorer when verification is done ──────────────────────────

    def report_result(self, task_id: str, mine_id: str,
                      outcome: str, confidence: float):
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
        self.busy = False
        self.current_task_id = None

    # ── State update (called by mission_logic_node) ───────────────────────────

    def update_state(self, state: str):
        self.my_state = state


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
