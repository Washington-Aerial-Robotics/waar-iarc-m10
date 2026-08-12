"""Synchronous, fail-closed ESP32 session used by the ROS node and tests."""

from __future__ import annotations

import time
from typing import Tuple

from .protocol import (
    COM_FAILURE,
    COM_REPLY_TELEMETRY,
    COM_REQUEST_TELEMETRY,
    COM_SET_ACTUATION,
    COM_SET_FLIGHTMODE,
    COM_SET_GPSORIGIN,
    COM_SET_TRAJECTORY,
    COM_SET_TRAJSETPT,
    COM_SUCCESS,
    FLIGHTPATH_LAND,
    FLAG_ACTUATION,
    Packet,
    ReplyMatcher,
    Telemetry,
    build_packet,
    pack_setpoint,
    parse_packet,
    unpack_telemetry,
)
from .transport import BaseTransport, TransportError


class ProtocolError(RuntimeError):
    pass


class Esp32Session:
    def __init__(
        self,
        transport: BaseTransport,
        device_id: str = "U",
        sender_id: str = "P",
        timeout_s: float = 0.3,
    ) -> None:
        self.transport = transport
        self.device_id = device_id
        self.sender_id = sender_id
        self.timeout_s = timeout_s
        self._next_message_id = 1
        self.sequence = 0
        self.last_telemetry: Telemetry | None = None
        self.last_telemetry_monotonic: float | None = None

    def connect(self) -> None:
        self.transport.connect()

    def close(self) -> None:
        self.transport.close()

    def _message_id(self) -> int:
        value = self._next_message_id
        self._next_message_id = 1 if value == 255 else value + 1
        return value

    def _transact(
        self, message_type: int, payload: bytes, expected_type: int
    ) -> Packet:
        message_id = self._message_id()
        request = build_packet(
            self.device_id, self.sender_id, message_type, message_id, payload
        )
        try:
            response = parse_packet(self.transport.transact(request, self.timeout_s))
        except (OSError, ValueError, TransportError) as exc:
            raise ProtocolError(str(exc)) from exc
        matcher = ReplyMatcher(self.device_id, self.sender_id)
        if response.message_type == COM_FAILURE and matcher.matches(
            response, message_id, COM_FAILURE
        ):
            raise ProtocolError(f"ESP32 rejected command 0x{message_type:02x}")
        if not matcher.matches(response, message_id, expected_type):
            raise ProtocolError(
                "unexpected ESP32 reply "
                f"to={response.to_id:#x} from={response.from_id:#x} "
                f"type={response.message_type:#x} id={response.message_id:#x}"
            )
        return response

    def telemetry(self) -> Telemetry:
        packet = self._transact(COM_REQUEST_TELEMETRY, b"", COM_REPLY_TELEMETRY)
        result = unpack_telemetry(packet.payload)
        self.last_telemetry = result
        self.last_telemetry_monotonic = time.monotonic()
        return result

    def latch_origin(self) -> None:
        self._transact(COM_SET_GPSORIGIN, b"", COM_SUCCESS)
        # A successful, disarmed origin latch establishes a new stream session.
        self.sequence = 0

    def setpoint(
        self,
        position: Tuple[float, float, float],
        yaw_kaf: float,
        velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> int:
        if self.sequence >= 0xFFFFFFFF:
            raise ProtocolError("setpoint sequence exhausted; land and prepare again")
        sequence = self.sequence + 1
        payload = pack_setpoint(sequence, position, yaw_kaf, velocity)
        self._transact(COM_SET_TRAJSETPT, payload, COM_SUCCESS)
        self.sequence = sequence
        return sequence

    def flight_mode(self, mode: int) -> None:
        self._transact(COM_SET_FLIGHTMODE, bytes((mode & 0xFF, 0)), COM_SUCCESS)

    def actuation(self, armed: bool) -> None:
        self._transact(COM_SET_ACTUATION, b"\xff" if armed else b"\x00", COM_SUCCESS)

    def disarm(self) -> Telemetry:
        """Disable actuation and prove it with telemetry.

        The Pi protocol deliberately does not send flight-mode 0x10. That mode
        is reserved by the firmware and is not its disarm contract.
        """
        self.actuation(False)
        telemetry = self.telemetry()
        if telemetry.has(FLAG_ACTUATION):
            raise ProtocolError("ESP32 did not confirm actuation disabled")
        return telemetry

    def land(self) -> None:
        self._transact(COM_SET_TRAJECTORY, bytes((FLIGHTPATH_LAND,)), COM_SUCCESS)
