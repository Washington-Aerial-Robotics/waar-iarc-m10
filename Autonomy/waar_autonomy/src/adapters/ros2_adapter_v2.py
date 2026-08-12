"""
ros2_adapter_v2.py
-------------------
Bridges waar_autonomy's single-drone `Simulator` to the mas_coordinator ROS2
topics, driving the whole team from one node.

Scope note: mas_coordinator's docs (mas_coordinator/CLAUDE.md,
mas_coordinator/README.md) describe this file as bridging a class called
`BaselineLoop`. That class does not exist anywhere in this repo. The actual
single-drone planner here is `application.simulator.Simulator`
(WorldModel + DroneState + GroundTruthPort, ticked through
observe -> corridor -> certify -> frontier). This adapter is written against
that real Simulator, not the documented-but-absent BaselineLoop -- see
waar_autonomy/README.md "Known limitations" for what that substitution does
and does not cover.

Requires `waar_autonomy/src` on PYTHONPATH:
    export PYTHONPATH=/path/to/waar_autonomy/src:$PYTHONPATH
    python3 waar_autonomy/src/adapters/ros2_adapter_v2.py
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mas_interfaces.msg import PoseBeacon, MineBelief, TaskResult

from domain.types import HAZARD
from domain.world_model import WorldModel
from domain.drone_state import DroneState
from experiments.config import Config
from infrastructure.env.map_factory import build_ground_truth_map
from adapters.sim_ground_truth_adapter import SimGroundTruthAdapter
from application.simulator import Simulator
from use_cases.update_world_model import observe_block


# ── Coordinate conversion ────────────────────────────────────────────────────
# Pure functions expected by mas_coordinator/tests/test_state_machine.py::
# TestCoordConversion. `coord` is (row, col) block indices; (x, y) is metres
# in the ROS2/arena frame.
#
# CELL_SIZE is a placeholder. waar_autonomy's block grid has no inherent
# physical scale (Config defaults to a 20x15 block test arena), while
# mas_coordinator assumes a real 91.44m x 24.38m IARC arena. Reporting block
# coordinates as metres 1:1 means a fully-explored simulated arena only ever
# covers a ~20m x 15m box in ROS2's frame -- geofence/collision guards in
# mission_logic_node, which are sized for the real arena, will not see
# meaningful bounds. Fixing this needs an explicit arena-scale decision, not
# a guess baked into an adapter. See README "Known limitations".
CELL_SIZE = 1.0


def coord_to_xy(coord: tuple[int, int]) -> tuple[float, float]:
    row, col = coord
    return col * CELL_SIZE, row * CELL_SIZE


def xy_to_coord(x: float, y: float) -> tuple[int, int]:
    return int(y / CELL_SIZE), int(x / CELL_SIZE)


# ── Per-drone runtime state ──────────────────────────────────────────────────

@dataclass
class _DroneRuntime:
    drone_id: str
    sim: Simulator
    seq: int = 0
    held: bool = False                 # HOLD_POSITION mission_cmd
    finished: bool = False             # LAND_AND_SUBMIT mission_cmd
    active_task: Optional[dict] = None  # current task_cmd JSON, if any
    reported_mines: set = field(default_factory=set)  # fine cells already published


class Ros2ExplorerNode(Node):
    """Drives one `Simulator` per drone and bridges it to the MAS topics."""

    def __init__(self) -> None:
        super().__init__("ros2_adapter_v2")

        self.declare_parameter("drone_ids", ["d1", "d2", "d3", "d4"])
        self.declare_parameter("tick_hz", 2.0)
        self.declare_parameter("block_cols", Config.block_cols)
        self.declare_parameter("block_rows", Config.block_rows)
        self.declare_parameter("n_hazards", Config.n_hazards)
        self.declare_parameter("seed", 42)
        self.declare_parameter("inflation_radius", Config.inflation_radius)
        self.declare_parameter("unknown_cost", Config.unknown_cost)
        self.declare_parameter("min_clearance_cells", Config.min_clearance_cells)
        self.declare_parameter("min_coverage_ratio", Config.min_coverage_ratio)
        self.declare_parameter("w_cert", Config.w_cert)

        drone_ids = list(self.get_parameter("drone_ids").value)
        tick_hz = float(self.get_parameter("tick_hz").value)
        seed = int(self.get_parameter("seed").value)
        cfg = Config(
            block_cols=int(self.get_parameter("block_cols").value),
            block_rows=int(self.get_parameter("block_rows").value),
            n_hazards=int(self.get_parameter("n_hazards").value),
            inflation_radius=float(self.get_parameter("inflation_radius").value),
            unknown_cost=float(self.get_parameter("unknown_cost").value),
            min_clearance_cells=float(self.get_parameter("min_clearance_cells").value),
            min_coverage_ratio=float(self.get_parameter("min_coverage_ratio").value),
            w_cert=float(self.get_parameter("w_cert").value),
        )

        self._drones: Dict[str, _DroneRuntime] = {}
        self._pub_pose: Dict[str, object] = {}
        self._pub_mine: Dict[str, object] = {}

        # Shared team topics -- one publisher for the whole node, not per drone.
        self._pub_beacon = self.create_publisher(PoseBeacon, "/team/pose_beacon", 20)
        self._pub_result = self.create_publisher(TaskResult, "/team/task_result", 10)

        for i, drone_id in enumerate(drone_ids):
            # Each drone gets its own WorldModel/DroneState/ground truth --
            # fixes the "4 drones share one WorldModel" limitation documented
            # in mas_coordinator/CLAUDE.md (frontier exhaustion after ~1 drone
            # covers the area).
            wm = WorldModel(cfg.block_cols, cfg.block_rows)
            drone = DroneState(cfg.block_cols, cfg.block_rows)
            gt_map = build_ground_truth_map(
                cfg.block_cols, cfg.block_rows, cfg.n_hazards, seed + i)
            gt = SimGroundTruthAdapter(gt_map)
            sim = Simulator(
                world=wm, drone=drone, gt=gt,
                inflation_radius=cfg.inflation_radius,
                unknown_cost=cfg.unknown_cost,
                min_clearance_cells=cfg.min_clearance_cells,
                min_coverage_ratio=cfg.min_coverage_ratio,
                w_cert=cfg.w_cert,
            )
            self._drones[drone_id] = _DroneRuntime(drone_id=drone_id, sim=sim)

            self._pub_pose[drone_id] = self.create_publisher(
                PoseStamped, f"/{drone_id}/pose", 10)
            self._pub_mine[drone_id] = self.create_publisher(
                MineBelief, f"/{drone_id}/mine_candidates", 10)

            self.create_subscription(
                String, f"/{drone_id}/task_cmd",
                self._make_task_cmd_handler(drone_id), 10)
            self.create_subscription(
                String, f"/{drone_id}/mission_cmd",
                self._make_mission_cmd_handler(drone_id), 10)

        self.create_timer(1.0 / tick_hz, self._tick_all)
        self.get_logger().info(
            f"ros2_adapter_v2 ready | drones={drone_ids} | "
            f"blocks={cfg.block_cols}x{cfg.block_rows} | tick_hz={tick_hz}")

    # ── Command handlers ────────────────────────────────────────────────────

    def _make_task_cmd_handler(self, drone_id: str):
        def _handler(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warn(f"[{drone_id}] bad task_cmd JSON: {msg.data!r}")
                return
            self._drones[drone_id].active_task = data
            self.get_logger().info(f"[{drone_id}] task_cmd: {data}")
        return _handler

    def _make_mission_cmd_handler(self, drone_id: str):
        def _handler(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except (json.JSONDecodeError, TypeError):
                self.get_logger().warn(f"[{drone_id}] bad mission_cmd JSON: {msg.data!r}")
                return
            rt = self._drones[drone_id]
            cmd = data.get("cmd")
            # Only HOLD_POSITION and LAND_AND_SUBMIT are actuated (mas_coordinator/
            # CLAUDE.md "Next Steps" #4, minimum bar). SWEEP_SECTOR, FILL_GAPS,
            # VERIFY_PATH, STANDBY_FOR_TASK, and AWAIT_PATH_VERIFY all require
            # sector-aware exploration that the Simulator's frontier scorer does
            # not implement -- they are logged only. See README "Known limitations".
            if cmd == "HOLD_POSITION":
                rt.held = True
            elif cmd == "LAND_AND_SUBMIT":
                rt.finished = True
            else:
                rt.held = False
            self.get_logger().info(f"[{drone_id}] mission_cmd: {data}")
        return _handler

    # ── Main tick ────────────────────────────────────────────────────────────

    def _tick_all(self) -> None:
        for rt in self._drones.values():
            if rt.finished:
                continue
            self._tick_one(rt)

    def _tick_one(self, rt: _DroneRuntime) -> None:
        task = rt.active_task

        if task is not None and task.get("task_type") == "VERIFY_TAG":
            self._step_toward_task(rt, task)
        elif not rt.held:
            rt.sim.tick()

        self._publish_pose(rt)
        self._publish_new_mines(rt)

    def _step_toward_task(self, rt: _DroneRuntime, task: dict) -> None:
        """Move directly toward a VERIFY_TAG target instead of exploring."""
        target_x = task.get("target_x")
        target_y = task.get("target_y")
        if target_x is None or target_y is None:
            return

        sim = rt.sim
        target_row, target_col = xy_to_coord(target_x, target_y)
        bx, by = sim.drone.block

        if (bx, by) == (target_col, target_row):
            self._report_task_result(rt, task)
            rt.active_task = None
            return

        step_x = 1 if target_col > bx else (-1 if target_col < bx else 0)
        step_y = 1 if target_row > by else (-1 if target_row < by else 0)
        nbx, nby = bx + step_x, by + step_y
        sim.drone.move_to(nbx, nby)
        observe_block(sim.world, sim.gt, nbx, nby, sim.inflation_radius)

    def _report_task_result(self, rt: _DroneRuntime, task: dict) -> None:
        """
        Publish TaskResult directly to /team/task_result. p2p_task_node runs
        in a separate ROS2 process, so this has to go over the topic rather
        than calling report_result() as a Python method (mas_coordinator/
        CLAUDE.md Known Bug #4 / Next Steps #3).
        """
        task_id = task["task_id"]
        prefix = "verify_"
        mine_id = task_id[len(prefix):] if task_id.startswith(prefix) else task_id

        msg = TaskResult()
        msg.task_id = task_id
        msg.executor_id = rt.drone_id
        msg.outcome = "confirmed"
        msg.mine_id = mine_id
        msg.confidence = 1.0
        msg.stamp = self.get_clock().now().to_msg()
        self._pub_result.publish(msg)
        self.get_logger().info(
            f"[{rt.drone_id}] VERIFY_TAG {task_id} complete -> reported")

    def _publish_pose(self, rt: _DroneRuntime) -> None:
        bx, by = rt.sim.drone.block
        x, y = coord_to_xy((by, bx))
        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 1.5
        pose.pose.orientation.w = 1.0
        self._pub_pose[rt.drone_id].publish(pose)

        beacon = PoseBeacon()
        beacon.drone_id = rt.drone_id
        beacon.x = x
        beacon.y = y
        beacon.z = 1.5
        beacon.heading_deg = 0.0
        # No sm_state bridge exists yet (mas_coordinator/CLAUDE.md Known Bug #1):
        # mission_logic_node never tells this adapter its real state machine
        # state. This is a local guess based on whether a task is active, not
        # the authoritative state.
        beacon.state = "VERIFY_TAG" if rt.active_task else "SURVEY"
        beacon.battery_pct = 100.0
        beacon.stamp = now
        self._pub_beacon.publish(beacon)

    def _publish_new_mines(self, rt: _DroneRuntime) -> None:
        """
        Simulator's sensing is binary (a fine cell is HAZARD or it isn't) --
        there is no continuous hazard_evidence confidence score to threshold
        against, unlike what mas_coordinator/CLAUDE.md describes. Every
        newly-detected hazard cell is published once as a full-confidence
        candidate instead.
        """
        world = rt.sim.world
        fine_x, fine_y = rt.sim.drone.fine

        # Scan only the neighbourhood just observed, not the whole map.
        for dx in range(-4, 5):
            for dy in range(-4, 5):
                fx, fy = fine_x + dx, fine_y + dy
                if not (0 <= fx < world.fine_cols and 0 <= fy < world.fine_rows):
                    continue
                if world.detected[fx, fy] != HAZARD:
                    continue

                key = (fx, fy)
                if key in rt.reported_mines:
                    continue
                rt.reported_mines.add(key)
                rt.seq += 1

                x, y = coord_to_xy((fy, fx))
                msg = MineBelief()
                msg.mine_id = f"m_{rt.drone_id}_{rt.seq}"
                msg.x = x
                msg.y = y
                msg.confidence = 1.0
                msg.status = "candidate"
                msg.last_updated_by = rt.drone_id
                msg.seq = rt.seq
                msg.stamp = self.get_clock().now().to_msg()
                self._pub_mine[rt.drone_id].publish(msg)
                self.get_logger().info(
                    f"[{rt.drone_id}] mine candidate {msg.mine_id} "
                    f"at ({x:.1f}, {y:.1f})")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ros2ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
