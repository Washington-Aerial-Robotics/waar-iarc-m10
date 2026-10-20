"""Exact little-endian KAF autonomy wire protocol.

This module intentionally contains no ROS imports, making protocol compatibility
testable on a development machine and on the Ubuntu flight computer.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Optional, Tuple


HEADER = struct.Struct("<BBBB")
SETPOINT = struct.Struct("<I7f")
TELEMETRY = struct.Struct("<BBBBI3f3f4f3ff")

COM_SUCCESS = 0x3C
COM_FAILURE = 0x3D
COM_SET_ACTUATION = 0x4A
COM_SET_FLIGHTMODE = 0x4C
COM_SET_TRAJECTORY = 0x5B
COM_SET_TRAJSETPT = 0x5D
COM_SET_GPSORIGIN = 0x64
COM_REQUEST_TELEMETRY = 0x65
COM_REPLY_TELEMETRY = 0x25

CMD_IDLE_MODE = 0x00
CMD_NOMINAL_MODE = 0x08
CMD_NULL_MODE = 0x10
CMD_DESCENT_MODE = 0x18
NULL_MODE = 0
POS_SETPOINT_MODE = 6
FLIGHTPATH_LAND = 2

FLAG_POSITION_VALID = 0x01
FLAG_ORIGIN_SET = 0x02
FLAG_ACTUATION = 0x04
FLAG_PI_STREAM = 0x08
FLAG_SETPOINT_FRESH = 0x10
FLAG_ATTITUDE_VALID = 0x20
FLAG_CONTROL_CALIBRATED = 0x40
FLAG_BATTERY_VALID = 0x80


def _id_byte(value: str | int) -> int:
    if isinstance(value, int) and 0 <= value <= 255:
        return value
    if isinstance(value, str):
        try:
            encoded = value.encode("ascii")
        except UnicodeEncodeError:
            encoded = b""
        if len(encoded) == 1:
            return encoded[0]
    raise ValueError("protocol IDs must be one ASCII character or a byte")


@dataclass(frozen=True)
class Packet:
    to_id: int
    from_id: int
    message_type: int
    message_id: int
    payload: bytes


def build_packet(
    to_id: str | int,
    from_id: str | int,
    message_type: int,
    message_id: int,
    payload: bytes = b"",
) -> bytes:
    return HEADER.pack(
        _id_byte(to_id), _id_byte(from_id), message_type & 0xFF, message_id & 0xFF
    ) + payload


def parse_packet(data: bytes) -> Packet:
    if len(data) < HEADER.size:
        raise ValueError("packet is shorter than its four-byte header")
    to_id, from_id, message_type, message_id = HEADER.unpack_from(data)
    return Packet(to_id, from_id, message_type, message_id, data[HEADER.size :])


def pack_setpoint(
    sequence: int,
    position: Tuple[float, float, float],
    yaw_kaf: float,
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> bytes:
    values = (*position, yaw_kaf, *velocity)
    if not 0 < sequence <= 0xFFFFFFFF:
        raise ValueError("setpoint sequence must be in [1, 2^32-1]")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("setpoint contains a non-finite value")
    return SETPOINT.pack(sequence, *values)


@dataclass(frozen=True)
class Telemetry:
    protocol_version: int
    flight_mode: int
    flags: int
    setpoint_sequence: int
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    quaternion_kaf: Tuple[float, float, float, float]
    angular_velocity_kaf: Tuple[float, float, float]
    battery_percent: float

    def has(self, flag: int) -> bool:
        return bool(self.flags & flag)


def unpack_telemetry(payload: bytes) -> Telemetry:
    if len(payload) != TELEMETRY.size:
        raise ValueError(f"telemetry payload must be {TELEMETRY.size} bytes")
    raw = TELEMETRY.unpack(payload)
    floats = raw[5:]
    if raw[0] != 1:
        raise ValueError(f"unsupported telemetry protocol version {raw[0]}")
    if raw[3] != 0:
        raise ValueError("telemetry reserved byte is non-zero")
    if not all(math.isfinite(value) for value in floats):
        raise ValueError("telemetry contains a non-finite value")
    quaternion = tuple(raw[11:15])
    norm = math.sqrt(sum(value * value for value in quaternion))
    if raw[2] & FLAG_ATTITUDE_VALID and not 0.99 <= norm <= 1.01:
        raise ValueError("valid telemetry attitude is not a unit quaternion")
    if raw[2] & FLAG_BATTERY_VALID and not 0.0 <= raw[18] <= 100.0:
        raise ValueError("valid telemetry battery percentage is outside [0, 100]")
    return Telemetry(
        protocol_version=raw[0],
        flight_mode=raw[1],
        flags=raw[2],
        setpoint_sequence=raw[4],
        position=tuple(raw[5:8]),
        velocity=tuple(raw[8:11]),
        quaternion_kaf=quaternion,
        angular_velocity_kaf=tuple(raw[15:18]),
        battery_percent=raw[18],
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def ros_yaw_to_kaf(yaw_ros: float) -> float:
    """Convert ROS-base yaw to KAF-body yaw (right/forward/up body axes)."""
    return wrap_angle(yaw_ros - math.pi / 2.0)


def kaf_yaw_to_ros(yaw_kaf: float) -> float:
    return wrap_angle(yaw_kaf + math.pi / 2.0)


def kaf_angular_velocity_to_ros(
    angular_velocity: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    wx, wy, wz = angular_velocity
    return wy, -wx, wz


def quaternion_multiply(
    left: Tuple[float, float, float, float],
    right: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def kaf_quaternion_to_ros_base(
    quaternion_kaf: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    """world<-KAF multiplied by KAF<-ROS (a +90 degree Z rotation)."""
    half = math.pi / 4.0
    result = quaternion_multiply(
        quaternion_kaf, (0.0, 0.0, math.sin(half), math.cos(half))
    )
    norm = math.sqrt(sum(value * value for value in result))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("invalid KAF attitude quaternion")
    return tuple(value / norm for value in result)


def yaw_from_quaternion(q: Tuple[float, float, float, float]) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class ReplyMatcher:
    """Matches the one in-flight request; unsolicited/incorrect replies fail closed."""

    def __init__(self, device_id: str | int, sender_id: str | int) -> None:
        self.device_id = _id_byte(device_id)
        self.sender_id = _id_byte(sender_id)

    def matches(
        self, packet: Packet, message_id: int, expected_type: int
    ) -> bool:
        return (
            packet.to_id == self.sender_id
            and packet.from_id == self.device_id
            and packet.message_id == (message_id & 0xFF)
            and packet.message_type == expected_type
        )
