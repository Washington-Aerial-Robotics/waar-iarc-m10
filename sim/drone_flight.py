"""
KAF drone flight model for mission sim.

Mirrors:
  - Flight modes in ESP32/KAF_Drone/src/communication.h
  - Motor mixing in Ground Station esp32_split2/lib/controllers/drone_controller.dart
  - MOTOR_SETPOINT_MODE path in flight_task.cpp (direct motor → output)

Exploration uses roll/pitch/yaw/throttle like the ground station, not grid teleport.
Position/trajectory modes match firmware stubs (no closed-loop autopilot on hardware yet).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# communication.h
NULL_MODE = 0x00
CALIBRATION_MODE = 0x01
MOTOR_SETPOINT_MODE = 0x02
POS_SETPOINT_MODE = 0x03
TRAJECTORY_MODE = 0x04

# Ground station control loop period
DEFAULT_CONTROL_DT_S = 0.25


def _curve_signed(value: float) -> float:
    clamped = max(-1.0, min(1.0, value))
    sign = -1.0 if clamped < 0 else 1.0
    return sign * math.sqrt(abs(clamped))


def _curve_throttle(value: float) -> float:
    return math.sqrt(max(0.0, min(1.0, value)))


def mix_motors_from_sticks(
    throttle: float,
    pitch: float,
    roll: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    """Same mixer as DroneController._startControlLoop (quad X)."""
    shaped_throttle = _curve_throttle(throttle)
    shaped_pitch = _curve_signed(pitch)
    shaped_roll = _curve_signed(roll)
    shaped_yaw = _curve_signed(yaw)

    def clamp_motor(v: float) -> float:
        return max(0.0, min(1.0, v))

    m0 = clamp_motor(shaped_throttle + shaped_pitch + shaped_roll - shaped_yaw)
    m1 = clamp_motor(shaped_throttle + shaped_pitch - shaped_roll + shaped_yaw)
    m2 = clamp_motor(shaped_throttle - shaped_pitch + shaped_roll + shaped_yaw)
    m3 = clamp_motor(shaped_throttle - shaped_pitch - shaped_roll - shaped_yaw)
    return m0, m1, m2, m3


@dataclass
class DroneFlightModel:
    """
    3D world state: x/y downrange & across field, z altitude (m above ground).
    Yaw = heading (rad, +x downrange). Pitch/roll are body tilt (rad) from sticks.
    """

    x: float
    y: float
    z: float = 1.5
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    flight_mode: int = NULL_MODE
    motors_enabled: bool = False
    hover_throttle: float = 0.50
    control_dt_s: float = DEFAULT_CONTROL_DT_S
    motor_setpoint: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    _stick_throttle: float = 0.0
    _stick_pitch: float = 0.0
    _stick_roll: float = 0.0
    _stick_yaw: float = 0.0
    max_horiz_accel_m_s2: float = 1.4
    max_yaw_rate_rad_s: float = math.radians(72.0)
    max_tilt_rad: float = math.radians(28.0)
    max_climb_m_s2: float = 2.0
    vel_drag: float = 0.35
    min_z_m: float = 0.4
    max_z_m: float = 3.0

    def arm(self) -> None:
        self.motors_enabled = True
        self.flight_mode = MOTOR_SETPOINT_MODE
        self._stick_throttle = self.hover_throttle

    def disarm(self) -> None:
        self.motors_enabled = False
        self.flight_mode = NULL_MODE
        self.motor_setpoint = (0.0, 0.0, 0.0, 0.0)
        self._stick_throttle = 0.0
        self._stick_pitch = 0.0
        self._stick_roll = 0.0
        self._stick_yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

    def set_altitude_limits(self, min_z_m: float, max_z_m: float) -> None:
        self.min_z_m = min_z_m
        self.max_z_m = max_z_m
        self.z = max(min_z_m, min(max_z_m, self.z))

    def set_sticks(self, throttle: float, pitch: float, roll: float, yaw: float) -> None:
        if not self.motors_enabled:
            return
        self._stick_throttle = throttle
        self._stick_pitch = pitch
        self._stick_roll = roll
        self._stick_yaw = yaw
        self.flight_mode = MOTOR_SETPOINT_MODE
        self.motor_setpoint = mix_motors_from_sticks(throttle, pitch, roll, yaw)

    def set_motor_cmd(self, m0: float, m1: float, m2: float, m3: float) -> None:
        """COM_SET_MOTOR_CMD — bypass stick shaping."""
        if not self.motors_enabled:
            return
        self.flight_mode = MOTOR_SETPOINT_MODE
        self.motor_setpoint = (
            max(0.0, min(1.0, m0)),
            max(0.0, min(1.0, m1)),
            max(0.0, min(1.0, m2)),
            max(0.0, min(1.0, m3)),
        )

    def controls_step(
        self,
        *,
        field_x_m: float,
        field_y_m: float,
        margin_m: float,
    ) -> None:
        """One flight_task controlsStep + simple integration (motor mode only)."""
        dt = self.control_dt_s
        if self.flight_mode == NULL_MODE or not self.motors_enabled:
            self.vx *= max(0.0, 1.0 - self.vel_drag * dt)
            self.vy *= max(0.0, 1.0 - self.vel_drag * dt)
            self.vz *= max(0.0, 1.0 - self.vel_drag * dt)
            self._integrate_position(dt, field_x_m, field_y_m, margin_m)
            return

        if self.flight_mode == MOTOR_SETPOINT_MODE:
            pitch_stick = _curve_signed(self._stick_pitch)
            roll_stick = _curve_signed(self._stick_roll)
            yaw_stick = _curve_signed(self._stick_yaw)
            thrust = _curve_throttle(self._stick_throttle)
            hover_thrust = _curve_throttle(self.hover_throttle)
            ax_body = pitch_stick * self.max_horiz_accel_m_s2
            ay_body = roll_stick * self.max_horiz_accel_m_s2
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            ax = c * ax_body - s * ay_body
            ay = s * ax_body + c * ay_body
            self.vx += ax * dt
            self.vy += ay * dt
            self.yaw += yaw_stick * self.max_yaw_rate_rad_s * dt
            self.pitch = pitch_stick * self.max_tilt_rad
            self.roll = roll_stick * self.max_tilt_rad
            climb_cmd = (thrust - hover_thrust) * self.max_climb_m_s2 * 3.5
            self.vz += climb_cmd * dt
            drag = max(0.0, 1.0 - self.vel_drag * dt)
            self.vx *= drag
            self.vy *= drag
            self.vz *= drag
            self._integrate_position(dt, field_x_m, field_y_m, margin_m)
            return

        # POS / TRAJECTORY: firmware WIP — no motion in sim until implemented on ESP32.
        self._integrate_position(dt, field_x_m, field_y_m, margin_m)

    def _integrate_position(
        self,
        dt: float,
        field_x_m: float,
        field_y_m: float,
        margin_m: float,
    ) -> None:
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt
        self.z = max(self.min_z_m, min(self.max_z_m, self.z))
        if self.z <= self.min_z_m or self.z >= self.max_z_m:
            self.vz = 0.0
        self.x = max(margin_m, min(field_x_m - margin_m, self.x))
        self.y = max(margin_m, min(field_y_m - margin_m, self.y))
        if self.x <= margin_m or self.x >= field_x_m - margin_m:
            self.vx = 0.0
        if self.y <= margin_m or self.y >= field_y_m - margin_m:
            self.vy = 0.0


@dataclass
class SerpentinePatrol:
    """Autonomy that outputs GS-style sticks to sweep the field (until real mission code exists)."""

    field_x_m: float
    field_y_m: float
    margin_m: float
    lane_y: float
    phase_x_m: float = 0.0
    target_z_m: float = 1.5
    target_x: float = 0.0
    target_y: float = 0.0
    _serpentine_dir: int = 1

    def __post_init__(self) -> None:
        self.target_x = self.margin_m + 2.0 + self.phase_x_m
        self.target_y = self.lane_y

    def _advance_target(self) -> None:
        self.target_x += 3.0
        self.target_y += self._serpentine_dir * 1.2
        if self.target_y >= self.field_y_m - self.margin_m:
            self.target_y = self.field_y_m - self.margin_m
            self._serpentine_dir = -1
        elif self.target_y <= self.margin_m:
            self.target_y = self.margin_m
            self._serpentine_dir = 1
        if self.target_x > self.field_x_m - self.margin_m:
            self.target_x = self.margin_m + 2.0

    def sticks_for_pose(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        hover_throttle: float,
    ) -> tuple[float, float, float, float]:
        dx = self.target_x - x
        dy = self.target_y - y
        dist = math.hypot(dx, dy)
        if dist < 0.6:
            self._advance_target()
            dx = self.target_x - x
            dy = self.target_y - y
            dist = math.hypot(dx, dy)

        desired_yaw = math.atan2(dy, dx) if dist > 1e-3 else yaw
        yaw_err = math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw))

        pitch = min(1.0, dist / 4.0) * 0.55
        roll = max(-1.0, min(1.0, yaw_err * 1.2)) * 0.35
        yaw_stick = max(-1.0, min(1.0, yaw_err / math.pi)) * 0.45
        z_err = self.target_z_m - z
        throttle = hover_throttle + max(-0.22, min(0.22, z_err * 0.18))
        return throttle, pitch, roll, yaw_stick
