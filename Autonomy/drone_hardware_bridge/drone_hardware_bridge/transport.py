"""Single-owner TCP/serial transports for the ESP32 connection."""

from __future__ import annotations

from collections import deque
import socket
import threading
import time
from typing import Deque, Optional

from .protocol import HEADER, TELEMETRY, parse_packet


class TransportError(RuntimeError):
    pass


class BaseTransport:
    def connect(self) -> None:
        raise NotImplementedError

    def transact(self, packet: bytes, timeout_s: float) -> bytes:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class StreamTransport(BaseTransport):
    """One synchronized stream prevents multiple clients and packet coalescing."""

    def __init__(self, packet_gap_s: float = 0.06) -> None:
        self.packet_gap_s = packet_gap_s
        self._last_send = 0.0
        self._lock = threading.Lock()

    def _write(self, data: bytes) -> None:
        raise NotImplementedError

    def _read_exact(self, count: int, timeout_s: float) -> bytes:
        raise NotImplementedError

    def transact(self, packet: bytes, timeout_s: float = 0.25) -> bytes:
        with self._lock:
            wait = self.packet_gap_s - (time.monotonic() - self._last_send)
            if wait > 0:
                time.sleep(wait)
            self._write(packet)
            self._last_send = time.monotonic()
            request = parse_packet(packet)
            # Serial shares a UART with firmware diagnostics. Scan to the exact
            # response envelope instead of interpreting debug text as a packet.
            deadline = time.monotonic() + timeout_s
            header = bytearray()
            while time.monotonic() < deadline:
                header.extend(self._read_exact(1, max(0.001, deadline - time.monotonic())))
                if len(header) > HEADER.size:
                    del header[0]
                if len(header) != HEADER.size:
                    continue
                candidate = parse_packet(bytes(header))
                if (
                    candidate.to_id == request.from_id
                    and candidate.from_id == request.to_id
                    and candidate.message_id == request.message_id
                ):
                    break
            else:
                raise TransportError("timed out waiting for the matching ESP32 response")
            parsed = parse_packet(bytes(header))
            payload_size = TELEMETRY.size if parsed.message_type == 0x25 else 0
            payload = self._read_exact(
                payload_size, max(0.001, deadline - time.monotonic())
            ) if payload_size else b""
            return bytes(header) + payload


class TcpTransport(StreamTransport):
    def __init__(self, host: str, port: int, **kwargs) -> None:
        super().__init__(**kwargs)
        self.host = host
        self.port = port
        self._socket: Optional[socket.socket] = None

    def connect(self) -> None:
        if self._socket is None:
            self._socket = socket.create_connection((self.host, self.port), timeout=2.0)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _write(self, data: bytes) -> None:
        if self._socket is None:
            raise TransportError("TCP transport is not connected")
        self._socket.sendall(data)

    def _read_exact(self, count: int, timeout_s: float) -> bytes:
        if self._socket is None:
            raise TransportError("TCP transport is not connected")
        self._socket.settimeout(timeout_s)
        result = bytearray()
        while len(result) < count:
            chunk = self._socket.recv(count - len(result))
            if not chunk:
                raise TransportError("ESP32 closed the TCP connection")
            result.extend(chunk)
        return bytes(result)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None


class SerialTransport(StreamTransport):
    def __init__(self, port: str, baud: int, **kwargs) -> None:
        super().__init__(**kwargs)
        self.port = port
        self.baud = baud
        self._serial = None

    def connect(self) -> None:
        if self._serial is None:
            import serial

            self._serial = serial.Serial(self.port, self.baud, timeout=0.1)

    def _write(self, data: bytes) -> None:
        if self._serial is None:
            raise TransportError("serial transport is not connected")
        self._serial.write(data)
        self._serial.flush()

    def _read_exact(self, count: int, timeout_s: float) -> bytes:
        if self._serial is None:
            raise TransportError("serial transport is not connected")
        deadline = time.monotonic() + timeout_s
        result = bytearray()
        while len(result) < count and time.monotonic() < deadline:
            result.extend(self._serial.read(count - len(result)))
        if len(result) != count:
            raise TransportError(f"serial response timed out at {len(result)}/{count} bytes")
        return bytes(result)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None


class DryRunTransport(BaseTransport):
    """Deterministic fake ESP32. It records every packet and never arms implicitly."""

    def __init__(self, device_id: str = "U", sender_id: str = "P") -> None:
        self.device_id = ord(device_id)
        self.sender_id = ord(sender_id)
        self.sent: Deque[bytes] = deque()
        self.connected = False
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.quaternion = (0.0, 0.0, 0.0, 1.0)
        self.angular_velocity = (0.0, 0.0, 0.0)
        self.flight_mode = CMD_NULL_MODE | NULL_MODE
        self.flags = (
            FLAG_POSITION_VALID
            | FLAG_CONTROL_CALIBRATED
            | FLAG_ATTITUDE_VALID
            | FLAG_BATTERY_VALID
        )
        self.sequence = 0

    def connect(self) -> None:
        self.connected = True

    def transact(self, packet: bytes, timeout_s: float = 0.25) -> bytes:
        del timeout_s
        if not self.connected:
            raise TransportError("dry-run transport is not connected")
        request = parse_packet(packet)
        self.sent.append(packet)
        reply_type = COM_SUCCESS
        payload = b""
        if request.message_type == COM_SET_GPSORIGIN:
            self.flags |= FLAG_ORIGIN_SET
            self.sequence = 0
            self.position = (0.0, 0.0, 0.0)
            self.velocity = (0.0, 0.0, 0.0)
        elif request.message_type == COM_SET_TRAJSETPT:
            values = SETPOINT.unpack(request.payload)
            self.sequence = values[0]
            self.position = tuple(values[1:4])
            self.velocity = tuple(values[5:8])
            self.flags |= FLAG_PI_STREAM | FLAG_SETPOINT_FRESH
        elif request.message_type == COM_SET_FLIGHTMODE:
            self.flight_mode = request.payload[0]
        elif request.message_type == COM_SET_ACTUATION:
            if request.payload == b"\xff":
                self.flags |= FLAG_ACTUATION
            else:
                self.flags &= ~FLAG_ACTUATION
        elif request.message_type == COM_SET_TRAJECTORY:
            self.flight_mode = CMD_DESCENT_MODE | POS_SETPOINT_MODE
        elif request.message_type == COM_REQUEST_TELEMETRY:
            reply_type = COM_REPLY_TELEMETRY
            payload = TELEMETRY.pack(
                1,
                self.flight_mode,
                self.flags,
                0,
                self.sequence,
                *self.position,
                *self.velocity,
                *self.quaternion,
                *self.angular_velocity,
                100.0,
            )
        else:
            reply_type = COM_FAILURE
        return build_packet(
            self.sender_id,
            self.device_id,
            reply_type,
            request.message_id,
            payload,
        )

    def close(self) -> None:
        self.connected = False


# Imports kept below the classes to make the wire constants visibly local to
# the fake behavior and avoid a second, divergent constant table.
from .protocol import (  # noqa: E402
    CMD_DESCENT_MODE,
    CMD_NULL_MODE,
    COM_FAILURE,
    COM_REPLY_TELEMETRY,
    COM_REQUEST_TELEMETRY,
    COM_SET_ACTUATION,
    COM_SET_FLIGHTMODE,
    COM_SET_GPSORIGIN,
    COM_SET_TRAJECTORY,
    COM_SET_TRAJSETPT,
    COM_SUCCESS,
    FLAG_ACTUATION,
    FLAG_ATTITUDE_VALID,
    FLAG_BATTERY_VALID,
    FLAG_CONTROL_CALIBRATED,
    FLAG_ORIGIN_SET,
    FLAG_PI_STREAM,
    FLAG_POSITION_VALID,
    FLAG_SETPOINT_FRESH,
    NULL_MODE,
    POS_SETPOINT_MODE,
    SETPOINT,
    build_packet,
)
