from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass

import numpy as np

# Message types (match ESP32/KAF_Drone/src/communication.h)
COM_CMD = 0x40
COM_SET_ST_EST = COM_CMD | 0x20  # 0x60
COM_REPLY_ST_EST = 0x21
COM_REQUEST_ST_EST = COM_CMD | 0x21  # 0x61
COM_REQUEST_POS = COM_CMD | 0x23  # 0x63
COM_REPLY_POS = 0x23
COM_REQUEST_ATT = COM_CMD | 0x25  # 0x65
COM_REPLY_ATT = 0x25
COM_REQUEST_SENSORS = COM_CMD | 0x26  # 0x66
COM_REPLY_SENSORS = 0x26
COM_SUCCESS = 0x3A

APP_DEVICE_ID = 0x47
STATE_STRUCT_SIZE = 64  # 4 coordinates × 4 floats


# MPU6050 scaling (firmware stores raw int16; range 8g / 2000 dps per firmware.cpp)
ACCEL_LSB_PER_G = 4096.0
GYRO_LSB_PER_DPS = 16.384


def raw_imu_to_si(accel_raw: np.ndarray, gyro_raw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    accel_m_s2 = accel_raw / ACCEL_LSB_PER_G * 9.81
    gyro_rad_s = gyro_raw / GYRO_LSB_PER_DPS * (np.pi / 180.0)
    return accel_m_s2, gyro_rad_s


@dataclass
class ImuSample:
    timestamp: float
    accel_m_s2: np.ndarray  # x, y, z
    gyro_rad_s: np.ndarray  # x, y, z


@dataclass
class StateEstimate:
    position: np.ndarray
    velocity: np.ndarray
    orientation_euler: np.ndarray  # yaw, pitch, roll radians in x,y,z
    angular_velocity: np.ndarray


def pack_state(
    position: np.ndarray,
    velocity: np.ndarray,
    orientation_euler: np.ndarray,
    angular_velocity: np.ndarray,
) -> bytes:
    """Pack state struct: 4×coordinate (x,y,z,stdev) in little-endian float32."""
    parts = []
    for vec in (position, velocity, orientation_euler, angular_velocity):
        v = vec.astype(np.float64).reshape(3)
        parts.extend([float(v[0]), float(v[1]), float(v[2]), 0.0])
    return struct.pack("<16f", *parts)


def unpack_state(data: bytes) -> StateEstimate:
    floats = struct.unpack("<16f", data[:STATE_STRUCT_SIZE])
    return StateEstimate(
        position=np.array(floats[0:3], dtype=np.float64),
        velocity=np.array(floats[4:7], dtype=np.float64),
        orientation_euler=np.array(floats[8:11], dtype=np.float64),
        angular_velocity=np.array(floats[12:15], dtype=np.float64),
    )


def unpack_coord(data: bytes, offset: int = 0) -> np.ndarray:
    x, y, z, _ = struct.unpack_from("<4f", data, offset)
    return np.array([x, y, z], dtype=np.float64)


class Esp32Client:
    """TCP client for KAF drone communication protocol."""

    def __init__(
        self,
        host: str,
        port: int = 23,
        drone_id: int = 65,
        timeout_s: float = 0.25,
    ):
        self.host = host
        self.port = port
        self.drone_id = drone_id & 0xFF
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self._message_id = 0

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _next_message_id(self) -> int:
        self._message_id = (self._message_id + 1) & 0xFF
        return self._message_id

    def _connect(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        sock.settimeout(self.timeout_s)
        self._sock = sock
        return sock

    def _exchange(self, packet: bytes, min_response: int = 4) -> bytes | None:
        try:
            sock = self._connect()
            sock.sendall(packet)
            return sock.recv(256)
        except OSError:
            self.close()
            return None

    def _header(self, message_type: int, payload_len: int = 0) -> bytes:
        return bytes(
            [
                self.drone_id,
                APP_DEVICE_ID,
                message_type,
                self._next_message_id(),
            ]
        )

    def request_sensors(self) -> ImuSample | None:
        packet = self._header(COM_REQUEST_SENSORS)
        response = self._exchange(packet, min_response=36)
        if response is None or len(response) < 36:
            return None
        if response[2] != COM_REPLY_SENSORS:
            return None
        accel = unpack_coord(response, 4)
        gyro = unpack_coord(response, 20)
        accel, gyro = raw_imu_to_si(accel, gyro)
        return ImuSample(timestamp=time.time(), accel_m_s2=accel, gyro_rad_s=gyro)

    def request_state(self) -> StateEstimate | None:
        packet = self._header(COM_REQUEST_ST_EST)
        response = self._exchange(packet, min_response=4 + STATE_STRUCT_SIZE)
        if response is None or len(response) < 4 + STATE_STRUCT_SIZE:
            return None
        if response[2] != COM_REPLY_ST_EST:
            return None
        return unpack_state(response[4:])

    def request_position(self) -> np.ndarray | None:
        packet = self._header(COM_REQUEST_POS)
        response = self._exchange(packet, min_response=20)
        if response is None or len(response) < 20:
            return None
        if response[2] != COM_REPLY_POS:
            return None
        return unpack_coord(response, 4)

    def request_attitude_euler(self) -> np.ndarray | None:
        packet = self._header(COM_REQUEST_ATT)
        response = self._exchange(packet, min_response=20)
        if response is None or len(response) < 20:
            return None
        if response[2] != COM_REPLY_ATT:
            return None
        return unpack_coord(response, 4)

    def set_state_estimate(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        orientation_euler: np.ndarray,
        angular_velocity: np.ndarray,
    ) -> bool:
        payload = pack_state(position, velocity, orientation_euler, angular_velocity)
        packet = self._header(COM_SET_ST_EST) + payload
        response = self._exchange(packet, min_response=4)
        if response is None or len(response) < 3:
            return False
        return response[2] in (COM_SUCCESS, COM_REPLY_ST_EST)
