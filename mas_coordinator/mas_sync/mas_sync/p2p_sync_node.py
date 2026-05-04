"""
p2p_sync_node.py
----------------
Implements Scheme B sync window protocol:

  1. Listen to /team/pose_beacon → maintain neighbor graph
  2. When a neighbor enters range (R_enter), send SyncHello
  3. On SyncAck, enter SYNC_ACTIVE window (T_sync seconds)
  4. During window, exchange MineDelta (missing beliefs)
  5. Publish fused local_belief for the mission node to read

Parameters (ROS2 params, all overridable from launch file):
  drone_id          : str   - unique ID for this drone (e.g. "d1")
  r_enter           : float - neighbor detection radius in metres (default 8.0)
  r_exit            : float - neighbor removal radius in metres  (default 12.0)
  t_sync            : float - sync window duration in seconds    (default 5.0)
  hello_interval    : float - how often to try hello with each neighbor (s) (default 3.0)
  beacon_rate       : float - Hz for pose beacon publish (default 5.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
import time
from typing import Dict, Set

from mas_interfaces.msg import (
    PoseBeacon, SyncHello, SyncAck, MineDelta, MineBelief
)
from geometry_msgs.msg import PoseStamped   # source of local pose from explorer

from .belief_fusion import BeliefStore, msg_to_entry, entry_to_msg


# ── Neighbor state ────────────────────────────────────────────────────────────

class NeighborInfo:
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        self.x = 0.0
        self.y = 0.0
        self.state = "UNKNOWN"
        self.last_seen: float = 0.0          # monotonic time
        self.sync_active: bool = False
        self.sync_expires: float = 0.0       # monotonic time
        self.last_hello_sent: float = 0.0    # monotonic time
        self.known_mine_count: int = 0       # their count at last hello/ack


# ── Main node ─────────────────────────────────────────────────────────────────

class P2PSyncNode(Node):

    def __init__(self):
        super().__init__("p2p_sync_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id",       "d1")
        self.declare_parameter("r_enter",         8.0)
        self.declare_parameter("r_exit",          12.0)
        self.declare_parameter("t_sync",          5.0)
        self.declare_parameter("hello_interval",  3.0)
        self.declare_parameter("beacon_rate",     5.0)

        self.drone_id      = self.get_parameter("drone_id").value
        self.r_enter       = self.get_parameter("r_enter").value
        self.r_exit        = self.get_parameter("r_exit").value
        self.t_sync        = self.get_parameter("t_sync").value
        self.hello_interval= self.get_parameter("hello_interval").value
        beacon_rate        = self.get_parameter("beacon_rate").value

        # ── State ─────────────────────────────────────────────────────────────
        self.belief_store  = BeliefStore()
        self.neighbors: Dict[str, NeighborInfo] = {}
        self.my_x = 0.0
        self.my_y = 0.0
        self.my_z = 0.0
        self.my_heading = 0.0
        self.my_state   = "SURVEY"  # default to SURVEY so beacons are valid
        self.my_battery = 100.0
        self._seq        = 0   # monotonic seq for dedup cache

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_beacon = self.create_publisher(PoseBeacon,  "/team/pose_beacon",  10)
        self.pub_hello  = self.create_publisher(SyncHello,   "/team/sync_hello",   10)
        self.pub_ack    = self.create_publisher(SyncAck,     "/team/sync_ack",     10)
        self.pub_delta  = self.create_publisher(MineDelta,   "/team/mine_delta",   10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseBeacon, "/team/pose_beacon",
            self._on_pose_beacon, 20)

        self.create_subscription(
            SyncHello, "/team/sync_hello",
            self._on_sync_hello, 10)

        self.create_subscription(
            SyncAck, "/team/sync_ack",
            self._on_sync_ack, 10)

        self.create_subscription(
            MineDelta, "/team/mine_delta",
            self._on_mine_delta, 20)

        # Local pose from explorer_node
        self.create_subscription(
            PoseStamped, f"/{self.drone_id}/pose",
            self._on_local_pose, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / beacon_rate, self._publish_beacon)
        self.create_timer(1.0,               self._sync_tick)   # neighbor management

        self.get_logger().info(f"[{self.drone_id}] p2p_sync_node ready")

    # ── Local pose callback ───────────────────────────────────────────────────

    def _on_local_pose(self, msg: PoseStamped):
        """Update our own position from the explorer node."""
        self.my_x = msg.pose.position.x
        self.my_y = msg.pose.position.y
        self.my_z = msg.pose.position.z
        # Yaw from quaternion (simplified — good enough for a heading display)
        q = msg.pose.orientation
        self.my_heading = math.degrees(
            math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
        )

    # ── Beacon management ─────────────────────────────────────────────────────

    def _publish_beacon(self):
        msg = PoseBeacon()
        msg.drone_id    = self.drone_id
        msg.x           = self.my_x
        msg.y           = self.my_y
        msg.z           = self.my_z
        msg.heading_deg = float(self.my_heading)
        msg.state       = self.my_state
        msg.battery_pct = self.my_battery
        msg.stamp       = self.get_clock().now().to_msg()
        self.pub_beacon.publish(msg)

    def _on_pose_beacon(self, msg: PoseBeacon):
        """Update neighbor graph based on incoming beacons."""
        if msg.drone_id == self.drone_id:
            return  # ignore own beacon

        dist = math.hypot(msg.x - self.my_x, msg.y - self.my_y)
        now  = time.monotonic()

        if msg.drone_id not in self.neighbors:
            if dist < self.r_enter:
                # New neighbor in range
                n = NeighborInfo(msg.drone_id)
                n.x, n.y = msg.x, msg.y
                n.state = msg.state
                n.last_seen = now
                self.neighbors[msg.drone_id] = n
                self.get_logger().info(
                    f"[{self.drone_id}] Neighbor {msg.drone_id} entered range ({dist:.1f}m)")
        else:
            n = self.neighbors[msg.drone_id]
            n.x, n.y = msg.x, msg.y
            n.state = msg.state
            n.last_seen = now

            # Remove if too far
            if dist > self.r_exit:
                del self.neighbors[msg.drone_id]
                self.get_logger().info(
                    f"[{self.drone_id}] Neighbor {msg.drone_id} left range ({dist:.1f}m)")

    # ── Sync window protocol ──────────────────────────────────────────────────

    def _sync_tick(self):
        """
        Called every 1 s.
        - Expire timed-out sync windows
        - Send hello to neighbors we haven't synced with recently
        """
        now = time.monotonic()

        # Expire stale neighbors (haven't heard from them in 3 s)
        stale = [nid for nid, n in self.neighbors.items()
                 if now - n.last_seen > 3.0]
        for nid in stale:
            del self.neighbors[nid]
            self.get_logger().debug(f"[{self.drone_id}] Removed stale neighbor {nid}")

        # Expire sync windows
        for n in self.neighbors.values():
            if n.sync_active and now > n.sync_expires:
                n.sync_active = False

        # Send hello to neighbors whose sync window has expired
        for n in self.neighbors.values():
            if (not n.sync_active and
                    now - n.last_hello_sent > self.hello_interval):
                self._send_hello(n)
                n.last_hello_sent = now

    def _send_hello(self, neighbor: NeighborInfo):
        msg = SyncHello()
        msg.sender_id       = self.drone_id
        msg.target_id       = neighbor.drone_id
        msg.known_mine_count= self.belief_store.count()
        msg.stamp           = self.get_clock().now().to_msg()
        self.pub_hello.publish(msg)

    def _on_sync_hello(self, msg: SyncHello):
        """Someone sent us a hello — respond with ack and send our delta."""
        if msg.target_id != self.drone_id:
            return   # not for us

        sender = msg.sender_id
        if sender not in self.neighbors:
            return   # we don't see this drone in range (ignore)

        # Send ack
        ack = SyncAck()
        ack.sender_id       = self.drone_id
        ack.target_id       = sender
        ack.known_mine_count= self.belief_store.count()
        ack.stamp           = self.get_clock().now().to_msg()
        self.pub_ack.publish(ack)

        # Activate sync window
        n = self.neighbors[sender]
        n.sync_active  = True
        n.sync_expires = time.monotonic() + self.t_sync
        n.known_mine_count = msg.known_mine_count

        # Send our delta (beliefs they're probably missing)
        self._send_delta(sender, msg.known_mine_count)

    def _on_sync_ack(self, msg: SyncAck):
        """Our hello was acknowledged — activate window and send delta."""
        if msg.target_id != self.drone_id:
            return

        sender = msg.sender_id
        if sender not in self.neighbors:
            return

        n = self.neighbors[sender]
        n.sync_active  = True
        n.sync_expires = time.monotonic() + self.t_sync
        n.known_mine_count = msg.known_mine_count

        # Send our delta
        self._send_delta(sender, msg.known_mine_count)

    def _send_delta(self, target_id: str, their_count: int):
        """
        Publish a MineDelta containing beliefs the target probably lacks.
        We use known_mine_count as a rough watermark (MVP simplification).
        """
        delta_entries = self.belief_store.get_delta_since(their_count)
        if not delta_entries:
            return

        now_msg = self.get_clock().now().to_msg()
        delta = MineDelta()
        delta.sender_id = self.drone_id
        delta.ttl       = 2
        delta.stamp     = now_msg
        delta.beliefs   = [
            entry_to_msg(e, MineBelief, now_msg) for e in delta_entries
        ]
        self.pub_delta.publish(delta)
        self.get_logger().debug(
            f"[{self.drone_id}] Sent {len(delta_entries)} beliefs to {target_id}")

    def _on_mine_delta(self, msg: MineDelta):
        """Receive and merge a delta from another drone."""
        if msg.sender_id == self.drone_id:
            return   # ignore own messages

        entries = [msg_to_entry(b) for b in msg.beliefs]
        changed = self.belief_store.merge_batch(entries)

        if changed > 0:
            self.get_logger().debug(
                f"[{self.drone_id}] Merged {changed} new/updated beliefs "
                f"from {msg.sender_id}")

        # TTL-based relay: decrement and re-broadcast if still alive
        if msg.ttl > 1:
            relay = MineDelta()
            relay.sender_id = self.drone_id   # we are the relay
            relay.ttl       = msg.ttl - 1
            relay.stamp     = self.get_clock().now().to_msg()
            relay.beliefs   = msg.beliefs
            self.pub_delta.publish(relay)

    # ── Public API for other nodes ─────────────────────────────────────────────

    def update_state(self, state: str):
        """Called by mission_logic_node to update the state string in beacons."""
        self.my_state = state

    def add_local_mine(self, mine_id: str, x: float, y: float,
                       confidence: float, status: str = "candidate"):
        """
        Called when explorer_node detects a new mine candidate.
        Adds to local store; will be shared at next sync window.
        """
        from .belief_fusion import BeliefEntry
        self._seq += 1
        entry = BeliefEntry(
            mine_id=mine_id,
            x=x, y=y,
            confidence=confidence,
            status=status,
            last_updated_by=self.drone_id,
            seq=self._seq,
            stamp_sec=time.time(),
        )
        self.belief_store.merge(entry)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = P2PSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
