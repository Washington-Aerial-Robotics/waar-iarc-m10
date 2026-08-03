"""
stub_explorer.py
----------------
Simulates Kevin's explorer_node for one drone during integration testing.
No real planning or sensing — just enough to exercise the full MAS stack.

Behaviour
---------
- Publishes /{drone_id}/pose (PoseStamped) at 5 Hz.
  Moves the drone on a lawnmower (boustrophedon) pattern across its sector.

- After MINE_DELAY_S seconds, publishes ONE /{drone_id}/mine_candidates
  (MineBelief) at a random position inside the sector with confidence 0.6.

- Subscribes to /{drone_id}/mission_cmd and /{drone_id}/task_cmd.
  Logs every message received; no actuation.

Parameters
----------
drone_id      : str   unique identifier (default "d1")
drone_index   : int   0-based index (default 0)
sector_x_min  : float sector left edge   (default 0.0)
sector_x_max  : float sector right edge  (default 15.0)
sector_y_min  : float sector bottom edge (default 0.0)
sector_y_max  : float sector top edge    (default 15.0)
move_speed    : float metres per second  (default 0.5)
"""

import random
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mas_interfaces.msg import MineBelief

MINE_DELAY_S = 30.0   # seconds after node start before publishing mine candidate
POSE_RATE_HZ = 5.0    # Hz for pose publishing

# Row spacing for the lawnmower — roughly one drone-width apart
LANE_SPACING = 2.0    # metres between horizontal sweeps


class StubExplorer(Node):

    def __init__(self):
        super().__init__("stub_explorer")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id",     "d1")
        self.declare_parameter("drone_index",   0)
        self.declare_parameter("sector_x_min",  0.0)
        self.declare_parameter("sector_x_max", 15.0)
        self.declare_parameter("sector_y_min",  0.0)
        self.declare_parameter("sector_y_max", 15.0)
        self.declare_parameter("move_speed",    0.5)

        self.drone_id    = self.get_parameter("drone_id").value
        self.x_min       = self.get_parameter("sector_x_min").value
        self.x_max       = self.get_parameter("sector_x_max").value
        self.y_min       = self.get_parameter("sector_y_min").value
        self.y_max       = self.get_parameter("sector_y_max").value
        self.move_speed  = self.get_parameter("move_speed").value

        # ── Lawnmower state ───────────────────────────────────────────────────
        # Start at the sector origin; sweep right, step up, sweep left, …
        self._x          = self.x_min
        self._y          = self.y_min
        self._direction  = 1.0     # +1 = right, -1 = left
        self._dt         = 1.0 / POSE_RATE_HZ
        self._step_dist  = self.move_speed * self._dt

        # ── Mine candidate tracking ───────────────────────────────────────────
        self._mine_published = False
        self._start_time     = time.monotonic()
        self._mine_seq       = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_pose = self.create_publisher(
            PoseStamped, f"/{self.drone_id}/pose", 10)

        self.pub_mine = self.create_publisher(
            MineBelief, f"/{self.drone_id}/mine_candidates", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            String, f"/{self.drone_id}/mission_cmd",
            self._on_mission_cmd, 10)

        self.create_subscription(
            String, f"/{self.drone_id}/task_cmd",
            self._on_task_cmd, 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"[{self.drone_id}] stub_explorer ready | "
            f"sector=({self.x_min},{self.y_min})-({self.x_max},{self.y_max})")

    # ── Tick ──────────────────────────────────────────────────────────────────

    def _tick(self):
        self._move()
        self._publish_pose()
        self._maybe_publish_mine()

    # ── Lawnmower movement ────────────────────────────────────────────────────

    def _move(self):
        self._x += self._direction * self._step_dist

        # Hit the right wall → step up a lane, reverse
        if self._x >= self.x_max:
            self._x = self.x_max
            self._y += LANE_SPACING
            self._direction = -1.0

        # Hit the left wall → step up a lane, reverse
        elif self._x <= self.x_min:
            self._x = self.x_min
            self._y += LANE_SPACING
            self._direction = 1.0

        # Past the top → restart from bottom
        if self._y >= self.y_max:
            self._y = self.y_min
            self._x = self.x_min
            self._direction = 1.0

    # ── Pose publisher ────────────────────────────────────────────────────────

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = 1.5    # nominal flight altitude
        # Orientation left as identity (zero yaw is fine for a stub)
        msg.pose.orientation.w = 1.0
        self.pub_pose.publish(msg)

    # ── Mine candidate publisher ──────────────────────────────────────────────

    def _maybe_publish_mine(self):
        if self._mine_published:
            return
        if time.monotonic() - self._start_time < MINE_DELAY_S:
            return

        self._mine_published = True
        self._mine_seq += 1

        mine_x = random.uniform(self.x_min, self.x_max)
        mine_y = random.uniform(self.y_min, self.y_max)
        mine_id = f"m_{self.drone_id}_{self._mine_seq}"

        msg = MineBelief()
        msg.mine_id         = mine_id
        msg.x               = mine_x
        msg.y               = mine_y
        msg.confidence      = 0.6
        msg.status          = "candidate"
        msg.last_updated_by = self.drone_id
        msg.seq             = self._mine_seq
        msg.stamp           = self.get_clock().now().to_msg()

        self.pub_mine.publish(msg)
        self.get_logger().info(
            f"[{self.drone_id}] Published mine candidate {mine_id} "
            f"at ({mine_x:.1f}, {mine_y:.1f})")

    # ── Command subscribers ───────────────────────────────────────────────────

    def _on_mission_cmd(self, msg: String):
        self.get_logger().info(
            f"[{self.drone_id}] mission_cmd: {msg.data}")

    def _on_task_cmd(self, msg: String):
        self.get_logger().info(
            f"[{self.drone_id}] task_cmd: {msg.data}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = StubExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
