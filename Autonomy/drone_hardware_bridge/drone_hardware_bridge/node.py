"""ROS 2 single-drone physical bridge.

All motor-affecting actions require explicit services. Mission messages can
select targets only after `/prepare` and `/arm`; they can never arm the drone.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from mas_interfaces.msg import TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .controller import Esp32Session, ProtocolError
from .frame_math import quaternion_from_yaw, transform_pose, world_vector_to_body
from .grid_planner import Grid, GridPlanner
from .mission import CommandPlanner
from .safety import flight_time_expired, landing_action, landing_grounded
from .protocol import (
    CMD_NOMINAL_MODE,
    FLAG_ACTUATION,
    FLAG_ATTITUDE_VALID,
    FLAG_BATTERY_VALID,
    FLAG_CONTROL_CALIBRATED,
    FLAG_ORIGIN_SET,
    FLAG_PI_STREAM,
    FLAG_POSITION_VALID,
    FLAG_SETPOINT_FRESH,
    POS_SETPOINT_MODE,
    Telemetry,
    kaf_angular_velocity_to_ros,
    kaf_quaternion_to_ros_base,
    ros_yaw_to_kaf,
    yaw_from_quaternion,
)
from .transport import DryRunTransport, SerialTransport, TcpTransport


Point3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


class HardwareBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_hardware_bridge")
        defaults = {
            "drone_id": "d1", "dry_run": True, "auto_arm": False,
            "transport": "tcp", "esp_host": "192.168.1.240", "esp_port": 70,
            "esp_device_id": "U", "sender_id": "P", "serial_port": "/dev/ttyUSB0",
            "serial_baud": 115200, "packet_gap_s": 0.06, "request_timeout_s": 0.3,
            "control_rate_hz": 5.0, "telemetry_rate_hz": 2.0,
            "telemetry_timeout_s": 1.0, "pose_timeout_s": 1.0, "map_timeout_s": 3.0,
            "map_frame": "map", "odom_frame": "odom", "base_frame": "base_link",
            "arena_width": 91.44, "arena_height": 24.38,
            "arena_map_aligned": False, "occupied_threshold": 65,
            "unknown_is_blocked": True, "inflation_radius_m": 0.5,
            "coverage_spacing_m": 2.0, "arrival_tolerance_m": 0.35,
            "takeoff_height_m": 1.5, "max_climb_rate_mps": 0.5,
            "frame_alignment_tolerance_m": 2.0, "verification_dwell_s": 1.0,
            "verification_timeout_s": 8.0, "verification_proximity_m": 1.0,
            "landed_height_m": 0.12, "landed_velocity_mps": 0.15,
            "land_timeout_s": 30.0, "require_battery_valid": True,
            "minimum_battery_percent": 20.0, "max_flight_time_s": 420.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.drone_id = str(p("drone_id"))
        self.map_frame = str(p("map_frame"))
        self.odom_frame = str(p("odom_frame"))
        self.base_frame = str(p("base_frame"))
        self.dry_run = bool(p("dry_run"))
        self.telemetry_timeout_s = float(p("telemetry_timeout_s"))
        self.pose_timeout_s = float(p("pose_timeout_s"))
        self.map_timeout_s = float(p("map_timeout_s"))
        self.arrival_tolerance_m = float(p("arrival_tolerance_m"))
        self.takeoff_height_m = float(p("takeoff_height_m"))
        self.max_climb_rate_mps = float(p("max_climb_rate_mps"))
        self.frame_alignment_tolerance_m = float(p("frame_alignment_tolerance_m"))
        self.verification_dwell_s = float(p("verification_dwell_s"))
        self.verification_timeout_s = float(p("verification_timeout_s"))
        self.verification_proximity_m = float(p("verification_proximity_m"))
        self.landed_height_m = float(p("landed_height_m"))
        self.landed_velocity_mps = float(p("landed_velocity_mps"))
        self.land_timeout_s = float(p("land_timeout_s"))
        self.require_battery_valid = bool(p("require_battery_valid"))
        self.minimum_battery_percent = float(p("minimum_battery_percent"))
        self.max_flight_time_s = float(p("max_flight_time_s"))
        if not math.isfinite(self.max_flight_time_s) or self.max_flight_time_s < 0.0 or (
            not self.dry_run and self.max_flight_time_s <= 0.0
        ):
            raise ValueError(
                "max_flight_time_s must be positive on hardware; zero is dry-run only"
            )
        self.occupied_threshold = int(p("occupied_threshold"))
        self.unknown_is_blocked = bool(p("unknown_is_blocked"))
        self.inflation_radius_m = float(p("inflation_radius_m"))
        if bool(p("auto_arm")):
            self.get_logger().error("auto_arm is disabled: call the explicit /arm service")

        packet_gap = float(p("packet_gap_s"))
        if self.dry_run:
            transport = DryRunTransport(str(p("esp_device_id")), str(p("sender_id")))
        elif str(p("transport")).lower() == "tcp":
            transport = TcpTransport(str(p("esp_host")), int(p("esp_port")), packet_gap_s=packet_gap)
        elif str(p("transport")).lower() == "serial":
            transport = SerialTransport(str(p("serial_port")), int(p("serial_baud")), packet_gap_s=packet_gap)
        else:
            raise ValueError("transport must be tcp or serial")
        self.session = Esp32Session(
            transport, str(p("esp_device_id")), str(p("sender_id")),
            float(p("request_timeout_s")),
        )
        self.command_planner = CommandPlanner(
            float(p("arena_width")), float(p("arena_height")),
            float(p("coverage_spacing_m")), bool(p("arena_map_aligned")),
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._state = "DISCONNECTED"
        self._flight_ready = False
        self._latest_telemetry: Optional[Telemetry] = None
        self._latest_telemetry_time: Optional[float] = None
        self._map_grid: Optional[Grid] = None
        self._map_time: Optional[float] = None
        self._map_pose: Optional[tuple[Point3, Quaternion]] = None
        self._odom_pose: Optional[tuple[Point3, Quaternion]] = None
        self._pose_time: Optional[float] = None
        self._target_odom: Optional[tuple[Point3, float]] = None
        self._flight_altitude_map: Optional[float] = None
        self._ground_height_odom = 0.0
        self._last_setpoint_time: Optional[float] = None
        self._landing_started: Optional[float] = None
        self._armed_at: Optional[float] = None
        self._pending_verification = None
        self._battery_warning_printed = False

        ready_qos = QoSProfile(depth=1)
        ready_qos.reliability = ReliabilityPolicy.RELIABLE
        ready_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._ready_pub = self.create_publisher(Bool, f"/{self.drone_id}/flight_ready", ready_qos)
        self._pose_pub = self.create_publisher(PoseStamped, f"/{self.drone_id}/pose", 10)
        self._esp_odom_pub = self.create_publisher(Odometry, "/esp32/odometry", 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self._result_pub = self.create_publisher(TaskResult, "/team/task_result", 10)
        self._status_pub = self.create_publisher(String, f"/{self.drone_id}/planner_status", 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odometry, 20)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 5)
        self.create_subscription(String, f"/{self.drone_id}/mission_cmd", self._on_mission_command, 10)
        self.create_subscription(String, f"/{self.drone_id}/task_cmd", self._on_mission_command, 10)
        self.create_subscription(String, f"/{self.drone_id}/verification_result", self._on_verification, 10)
        for name, callback in (
            ("prepare", self._prepare), ("arm", self._arm),
            ("land", self._land), ("disarm", self._disarm),
        ):
            self.create_service(Trigger, f"/{self.drone_id}/{name}", callback)
        self.create_timer(1.0 / float(p("control_rate_hz")), self._control_tick)
        self.create_timer(1.0 / float(p("telemetry_rate_hz")), self._telemetry_tick)
        self.create_timer(0.5, self._publish_ready)
        self._publish_ready()

    def _publish_ready(self) -> None:
        msg = Bool()
        msg.data = self._flight_ready
        self._ready_pub.publish(msg)

    def _set_ready(self, value: bool) -> None:
        self._flight_ready = value
        self._publish_ready()

    def _status(self, text: str) -> None:
        msg = String()
        msg.data = json.dumps({"state": self._state, "detail": text})
        self._status_pub.publish(msg)

    @staticmethod
    def _q_tuple(message) -> Quaternion:
        return message.x, message.y, message.z, message.w

    def _transform(
        self, position: Point3, orientation: Quaternion, source: str, target: str
    ) -> tuple[Point3, Quaternion]:
        if source == target:
            return position, orientation
        transform = self._tf_buffer.lookup_transform(target, source, Time())
        t = transform.transform.translation
        r = transform.transform.rotation
        return transform_pose(position, orientation, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _on_odometry(self, msg: Odometry) -> None:
        source = msg.header.frame_id
        if not source:
            self.get_logger().error("Ignoring /odometry/filtered without frame_id")
            return
        position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        orientation = self._q_tuple(msg.pose.pose.orientation)
        try:
            self._map_pose = self._transform(position, orientation, source, self.map_frame)
            self._odom_pose = self._transform(position, orientation, source, self.odom_frame)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f"Pose frame conversion unavailable: {exc}")
            return
        self._pose_time = time.monotonic()
        output = PoseStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.map_frame
        pos, quat = self._map_pose
        output.pose.position.x, output.pose.position.y, output.pose.position.z = pos
        output.pose.orientation.x, output.pose.orientation.y = quat[0], quat[1]
        output.pose.orientation.z, output.pose.orientation.w = quat[2], quat[3]
        self._pose_pub.publish(output)

    def _on_map(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id != self.map_frame:
            self.get_logger().error(
                f"Rejecting occupancy grid in {msg.header.frame_id!r}; expected {self.map_frame!r}"
            )
            return
        q = msg.info.origin.orientation
        yaw = yaw_from_quaternion((q.x, q.y, q.z, q.w))
        try:
            self._map_grid = Grid(
                int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                msg.info.origin.position.x, msg.info.origin.position.y, yaw, tuple(msg.data),
            )
            self._map_time = time.monotonic()
        except ValueError as exc:
            self.get_logger().error(f"Rejecting invalid map: {exc}")

    def _fresh_pose(self) -> bool:
        return self._pose_time is not None and time.monotonic() - self._pose_time <= self.pose_timeout_s

    def _fresh_map(self) -> bool:
        return self._map_time is not None and time.monotonic() - self._map_time <= self.map_timeout_s

    def _planner(self) -> GridPlanner:
        if self._map_grid is None:
            raise ValueError("no occupancy map")
        return GridPlanner(
            self._map_grid, self.occupied_threshold,
            self.unknown_is_blocked, self.inflation_radius_m,
        )

    def _telemetry_policy(self, telemetry: Telemetry, for_arm: bool) -> Optional[str]:
        required = (
            FLAG_POSITION_VALID
            | FLAG_ORIGIN_SET
            | FLAG_ATTITUDE_VALID
            | FLAG_CONTROL_CALIBRATED
        )
        if (telemetry.flags & required) != required:
            return "position/origin/attitude/control calibration is not valid"
        if for_arm and (telemetry.flags & (FLAG_PI_STREAM | FLAG_SETPOINT_FRESH)) != (
            FLAG_PI_STREAM | FLAG_SETPOINT_FRESH
        ):
            return "setpoint stream is not active and fresh"
        if self.require_battery_valid:
            if not telemetry.has(FLAG_BATTERY_VALID):
                return "battery measurement is not valid"
            if telemetry.battery_percent < self.minimum_battery_percent:
                return "battery is below configured arm threshold"
        elif not telemetry.has(FLAG_BATTERY_VALID) and not self._battery_warning_printed:
            self._battery_warning_printed = True
            self.get_logger().warning(
                "Firmware has no valid battery measurement; an independent monitor/cutoff is required"
            )
        return None

    def _prepare(self, request, response):
        del request
        self._set_ready(False)
        if not self.command_planner.arena_map_aligned:
            response.success, response.message = False, "arena_map_aligned is not explicitly calibrated"
            return response
        if not self._fresh_pose() or not self._fresh_map() or self._odom_pose is None:
            response.success, response.message = False, "fresh map and localization are required"
            return response
        try:
            self.session.connect()
            before = self.session.telemetry()
            if before.has(FLAG_ACTUATION):
                raise ProtocolError("prepare is forbidden while actuation is enabled")
            if not before.has(FLAG_CONTROL_CALIBRATED):
                raise ProtocolError("flight control calibration/hover thrust is invalid")
            self.session.latch_origin()
            telemetry = self.session.telemetry()
            problem = self._telemetry_policy(telemetry, for_arm=False)
            if problem:
                raise ProtocolError(problem)
            odom_position = self._odom_pose[0]
            mismatch = math.dist(telemetry.position, odom_position)
            if mismatch > self.frame_alignment_tolerance_m:
                raise ProtocolError(
                    f"ESP local ENU and {self.odom_frame} differ by {mismatch:.2f}m"
                )
            yaw_ros = yaw_from_quaternion(kaf_quaternion_to_ros_base(telemetry.quaternion_kaf))
            self._ground_height_odom = telemetry.position[2]
            self._target_odom = (telemetry.position, yaw_ros)
            sequence = self.session.setpoint(telemetry.position, ros_yaw_to_kaf(yaw_ros))
            telemetry = self.session.telemetry()
            if telemetry.setpoint_sequence != sequence or self._telemetry_policy(telemetry, True):
                raise ProtocolError("ESP32 did not confirm the initial hold stream")
            self._latest_telemetry = telemetry
            self._latest_telemetry_time = time.monotonic()
            self._state = "PREPARED"
            response.success, response.message = True, "prepared and streaming hold; motors remain disarmed"
        except (ProtocolError, OSError, ValueError) as exc:
            self._state = "FAULT"
            response.success, response.message = False, str(exc)
        self._status(response.message)
        return response

    def _arm(self, request, response):
        del request
        if self._state != "PREPARED" or self._map_pose is None:
            response.success, response.message = False, "call prepare successfully before arm"
            return response
        if not self._fresh_pose() or not self._fresh_map():
            response.success, response.message = False, "localization/map became stale"
            return response
        try:
            telemetry = self.session.telemetry()
            problem = self._telemetry_policy(telemetry, for_arm=True)
            if problem:
                raise ProtocolError(problem)
            self._flight_altitude_map = self._map_pose[0][2] + self.takeoff_height_m
            self.command_planner.hold(self._map_pose[0][:2], "takeoff hold")
            # The existing hold stream is refreshed immediately before mode and actuation.
            if self._target_odom is None:
                raise ProtocolError("initial hold target is missing")
            self.session.setpoint(self._target_odom[0], ros_yaw_to_kaf(self._target_odom[1]))
            expected_mode = CMD_NOMINAL_MODE | POS_SETPOINT_MODE
            self.session.flight_mode(expected_mode)
            # Start the hard limit before issuing the motor-enable transaction,
            # never after a later telemetry round trip.
            self._armed_at = time.monotonic()
            self.session.actuation(True)
            telemetry = self.session.telemetry()
            if telemetry.flight_mode != expected_mode or not telemetry.has(FLAG_ACTUATION):
                raise ProtocolError("ESP32 did not confirm armed position mode")
            self._latest_telemetry = telemetry
            self._latest_telemetry_time = time.monotonic()
            current = self._map_pose[0][:2]
            current_yaw = yaw_from_quaternion(self._map_pose[1])
            # Only after arming do we select the higher desired altitude. The
            # control timer walks Z toward it at max_climb_rate_mps.
            self._target_odom = self._map_target_to_odom(current, current_yaw)
            self._state = "ARMED"
            self._set_ready(True)
            response.success, response.message = True, "armed; climb is rate-limited to takeoff height"
        except (ProtocolError, OSError, ValueError) as exc:
            self._best_effort_disarm()
            self._state = "FAULT"
            self._set_ready(False)
            response.success, response.message = False, str(exc)
        self._status(response.message)
        return response

    def _begin_land(self) -> None:
        self._set_ready(False)
        self.command_planner.plan.path.clear()
        self.session.land()
        self._state = "LANDING"
        self._landing_started = time.monotonic()
        self._armed_at = None

    def _land(self, request, response):
        del request
        try:
            if self._state in ("DISCONNECTED", "DISARMED"):
                response.success, response.message = True, "already disarmed"
            else:
                self._begin_land()
                response.success, response.message = True, "controlled landing accepted; disarm is telemetry-gated"
        except (ProtocolError, OSError) as exc:
            self._set_ready(False)
            response.success, response.message = False, str(exc)
        self._status(response.message)
        return response

    def _best_effort_disarm(self) -> bool:
        try:
            self.session.disarm()
            self._armed_at = None
            return True
        except Exception:
            return False

    def _disarm(self, request, response):
        del request
        self._set_ready(False)
        try:
            telemetry = self.session.disarm()
            self._latest_telemetry = telemetry
            self._latest_telemetry_time = time.monotonic()
            self._armed_at = None
            self._state = "DISARMED"
            response.success, response.message = True, "actuation disabled"
        except (ProtocolError, OSError) as exc:
            self._state = "FAULT"
            response.success, response.message = False, str(exc)
        self._status(response.message)
        return response

    def _on_mission_command(self, msg: String) -> None:
        if self._map_pose is None:
            return
        if self._state != "ARMED":
            self.command_planner.hold(self._map_pose[0][:2], "command received while not armed")
            return
        if not self._fresh_map() or not self._fresh_pose():
            self._fail_hold("stale map/localization")
            return
        plan = self.command_planner.command(msg.data, self._map_pose[0][:2], self._planner())
        self._status(f"mission plan {plan.mode}: {plan.reason}")
        if plan.mode == "LAND":
            try:
                self._begin_land()
            except ProtocolError as exc:
                self._state = "FAULT"
                self._status(str(exc))
        elif plan.mode == "HOLD":
            try:
                self._target_odom = self._map_target_to_odom(
                    self._map_pose[0][:2], yaw_from_quaternion(self._map_pose[1])
                )
            except (TransformException, ValueError) as exc:
                self._fail_hold(f"hold frame conversion unavailable: {exc}")

    def _fail_hold(self, reason: str) -> None:
        if self._map_pose is not None:
            self.command_planner.hold(self._map_pose[0][:2], reason)
        if self._latest_telemetry is not None and self._latest_telemetry.has(FLAG_ATTITUDE_VALID):
            q = kaf_quaternion_to_ros_base(self._latest_telemetry.quaternion_kaf)
            self._target_odom = (
                self._latest_telemetry.position,
                yaw_from_quaternion(q),
            )
        self._set_ready(False)
        self._status(f"fail-safe hold: {reason}")

    def _map_target_to_odom(self, point: tuple[float, float], yaw_map: float) -> tuple[Point3, float]:
        altitude = self._flight_altitude_map if self._flight_altitude_map is not None else self._map_pose[0][2]
        position, orientation = self._transform(
            (point[0], point[1], altitude), quaternion_from_yaw(yaw_map),
            self.map_frame, self.odom_frame,
        )
        return position, yaw_from_quaternion(orientation)

    def _control_tick(self) -> None:
        if self._state not in ("PREPARED", "ARMED") or self._target_odom is None:
            return
        now = time.monotonic()
        if self._state == "ARMED" and flight_time_expired(
            self._armed_at, now, self.max_flight_time_s
        ):
            try:
                self._begin_land()
                self._status(
                    f"maximum flight time ({self.max_flight_time_s:.1f}s) reached; landing"
                )
            except (ProtocolError, OSError) as exc:
                self._set_ready(False)
                self._state = "FAULT"
                self._status(f"maximum-flight-time LAND failed: {exc}")
            return
        if self._state == "ARMED" and (not self._fresh_map() or not self._fresh_pose()):
            self._fail_hold("stale map/localization")
        if self._state == "ARMED" and self._map_pose is not None:
            remaining = self.command_planner.plan.path
            if remaining:
                planner = self._planner()
                if not planner.path_is_free(remaining):
                    self._fail_hold("remaining path intersects occupied/unknown map")
                    remaining = []
            if remaining:
                current_xy = self._map_pose[0][:2]
                self.command_planner.reached(current_xy, self.arrival_tolerance_m)
                remaining = self.command_planner.plan.path
            if remaining:
                target = remaining[0]
                current = self._map_pose[0][:2]
                yaw = math.atan2(target[1] - current[1], target[0] - current[0])
                try:
                    position, yaw_odom = self._map_target_to_odom(target, yaw)
                    self._target_odom = (position, yaw_odom)
                except (TransformException, ValueError) as exc:
                    self._fail_hold(f"map-to-odom transform unavailable: {exc}")
            self._check_task_completion()

        position, yaw_ros = self._target_odom
        dt = 0.0 if self._last_setpoint_time is None else now - self._last_setpoint_time
        if self._latest_telemetry is not None and dt > 0:
            current_z = self._latest_telemetry.position[2]
            max_step = self.max_climb_rate_mps * dt
            z = max(current_z - max_step, min(current_z + max_step, position[2]))
            position = position[0], position[1], z
        try:
            self.session.setpoint(position, ros_yaw_to_kaf(yaw_ros))
            self._last_setpoint_time = now
        except ProtocolError as exc:
            self._state = "FAULT"
            self._set_ready(False)
            self._status(f"setpoint stream failed: {exc}")

    def _check_task_completion(self) -> None:
        task = self.command_planner.plan.task
        if task is None or task.arrived is None:
            return
        now = time.monotonic()
        if task.task_type == "VERIFY_PATH":
            self._publish_result(task.task_id, "", "confirmed", 1.0)
        elif task.task_type == "VERIFY_TAG":
            if self._pending_verification is not None and now - task.arrived >= self.verification_dwell_s:
                self._publish_result(*self._pending_verification)
                self._pending_verification = None
            elif now - task.arrived >= self.verification_timeout_s:
                self._publish_result(task.task_id, task.mine_id, "uncertain", 0.0)

    def _on_verification(self, msg: String) -> None:
        result = self.command_planner.verification_result(msg.data, self.verification_proximity_m)
        if result is None:
            return
        task = self.command_planner.plan.task
        if task is not None and task.arrived is not None and time.monotonic() - task.arrived >= self.verification_dwell_s:
            self._publish_result(*result)
        else:
            self._pending_verification = result

    def _publish_result(self, task_id: str, mine_id: str, outcome: str, confidence: float) -> None:
        msg = TaskResult()
        msg.task_id, msg.executor_id, msg.mine_id = task_id, self.drone_id, mine_id
        msg.outcome, msg.confidence = outcome, float(confidence)
        msg.stamp = self.get_clock().now().to_msg()
        self._result_pub.publish(msg)
        if self._map_pose is not None:
            self.command_planner.hold(self._map_pose[0][:2], f"task {task_id} complete")

    def _telemetry_tick(self) -> None:
        if self._state == "DISCONNECTED":
            return
        try:
            telemetry = self.session.telemetry()
        except ProtocolError as exc:
            self._set_ready(False)
            if self._state == "ARMED":
                self._state = "FAULT"
            self._status(f"telemetry failed: {exc}")
            return
        self._latest_telemetry = telemetry
        self._latest_telemetry_time = time.monotonic()
        self._publish_telemetry(telemetry)
        if self._state == "ARMED":
            problem = self._telemetry_policy(telemetry, for_arm=True)
            if problem or not telemetry.has(FLAG_ACTUATION):
                self._fail_hold(problem or "actuation dropped")
                self._state = "FAULT"
        elif self._state == "LANDING":
            grounded = landing_grounded(
                telemetry.position[2], telemetry.velocity[2], self._ground_height_odom,
                self.landed_height_m, self.landed_velocity_mps,
            )
            timed_out = self._landing_started is not None and time.monotonic() - self._landing_started >= self.land_timeout_s
            action = landing_action(telemetry.has(FLAG_ACTUATION), grounded, timed_out)
            if action == "ALREADY_DISARMED":
                self._state = "DISARMED"
                self._armed_at = None
                self._status("firmware reports landing/disarm complete")
            elif action == "DISARM":
                try:
                    telemetry = self.session.disarm()
                    self._latest_telemetry = telemetry
                    self._latest_telemetry_time = time.monotonic()
                    self._armed_at = None
                    self._state = "DISARMED"
                    self._status("landing complete and disarm confirmed")
                except ProtocolError as exc:
                    self._state = "FAULT"
                    self._status(f"grounded disarm was not confirmed: {exc}")
            elif action == "RETRY_LAND":
                try:
                    self.session.land()
                    self._landing_started = time.monotonic()
                    self._status("landing timeout while airborne; LAND retried without disarm")
                except ProtocolError as exc:
                    self._state = "FAULT"
                    self._status(f"landing retry failed; firmware watchdog remains responsible: {exc}")

    def _publish_telemetry(self, telemetry: Telemetry) -> None:
        stamp = self.get_clock().now().to_msg()
        orientation_valid = telemetry.has(FLAG_ATTITUDE_VALID)
        orientation = (
            kaf_quaternion_to_ros_base(telemetry.quaternion_kaf)
            if orientation_valid else (0.0, 0.0, 0.0, 1.0)
        )
        if telemetry.has(FLAG_POSITION_VALID) and orientation_valid:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.odom_frame, self.base_frame
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = telemetry.position
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = orientation[0], orientation[1]
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = orientation[2], orientation[3]
            body_velocity = world_vector_to_body(telemetry.velocity, orientation)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = body_velocity
            self._esp_odom_pub.publish(odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, self.base_frame
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = orientation
        if not orientation_valid:
            imu.orientation_covariance[0] = -1.0
        angular = kaf_angular_velocity_to_ros(telemetry.angular_velocity_kaf)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = angular
        imu.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(imu)

    def destroy_node(self):
        self._set_ready(False)
        if self._state in ("ARMED", "LANDING", "FAULT"):
            try:
                self.session.land()
            except Exception:
                pass
            # Shutdown does not prove touchdown. The firmware's acknowledged
            # LAND path/watchdog owns descent; never cut airborne actuation here.
        self.session.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
