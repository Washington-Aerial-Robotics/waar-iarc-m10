import pytest

from drone_hardware_bridge.controller import Esp32Session, ProtocolError
from drone_hardware_bridge.protocol import (
    CMD_NOMINAL_MODE, COM_REQUEST_TELEMETRY, COM_SET_ACTUATION,
    COM_SET_FLIGHTMODE, COM_SET_GPSORIGIN, COM_SET_TRAJSETPT,
    POS_SETPOINT_MODE, build_packet, parse_packet,
)
from drone_hardware_bridge.transport import BaseTransport, DryRunTransport


class WrongEnvelopeTransport(BaseTransport):
    def connect(self):
        pass

    def transact(self, packet, timeout_s):
        del timeout_s
        request = parse_packet(packet)
        return build_packet("P", "X", 0x3C, request.message_id)

    def close(self):
        pass


def test_prepare_stream_then_explicit_arm_order():
    transport = DryRunTransport()
    session = Esp32Session(transport)
    session.connect()
    session.latch_origin()
    session.setpoint((0.0, 0.0, 0.0), 0.0)
    # Nothing in origin/stream preparation arms actuation.
    assert [parse_packet(p).message_type for p in transport.sent] == [
        COM_SET_GPSORIGIN, COM_SET_TRAJSETPT,
    ]
    session.flight_mode(CMD_NOMINAL_MODE | POS_SETPOINT_MODE)
    session.actuation(True)
    assert [parse_packet(p).message_type for p in transport.sent][-2:] == [
        COM_SET_FLIGHTMODE, COM_SET_ACTUATION,
    ]


def test_origin_resets_sequence_and_first_setpoint_is_one():
    transport = DryRunTransport()
    session = Esp32Session(transport)
    session.connect()
    session.latch_origin()
    assert session.setpoint((0.0, 0.0, 0.0), 0.0) == 1
    assert session.setpoint((1.0, 0.0, 0.0), 0.0) == 2
    session.latch_origin()
    telemetry = session.telemetry()
    assert telemetry.position == (0.0, 0.0, 0.0)
    assert telemetry.velocity == (0.0, 0.0, 0.0)
    assert telemetry.setpoint_sequence == 0
    assert session.setpoint((0.0, 0.0, 0.0), 0.0) == 1


def test_unknown_command_fails_closed():
    transport = DryRunTransport()
    session = Esp32Session(transport)
    session.connect()
    with pytest.raises(ProtocolError):
        session._transact(0x7E, b"", 0x3C)


def test_wrong_reply_envelope_fails_closed():
    session = Esp32Session(WrongEnvelopeTransport())
    session.connect()
    with pytest.raises(ProtocolError, match="unexpected ESP32 reply"):
        session.latch_origin()


def test_pi_disarm_uses_actuation_only_and_never_forbidden_null_mode():
    transport = DryRunTransport()
    session = Esp32Session(transport)
    session.connect()
    session.actuation(True)
    transport.sent.clear()

    telemetry = session.disarm()

    packets = [parse_packet(raw) for raw in transport.sent]
    assert [packet.message_type for packet in packets] == [
        COM_SET_ACTUATION, COM_REQUEST_TELEMETRY,
    ]
    assert packets[0].payload == b"\x00"
    assert COM_SET_FLIGHTMODE not in [packet.message_type for packet in packets]
    assert not telemetry.has(0x04)


def test_complete_dry_run_session_requires_each_explicit_lifecycle_step():
    transport = DryRunTransport()
    session = Esp32Session(transport)
    session.connect()

    assert not session.telemetry().has(0x04)
    session.latch_origin()
    sequence = session.setpoint((0.0, 0.0, 1.5), 0.0)
    assert session.telemetry().setpoint_sequence == sequence
    session.flight_mode(CMD_NOMINAL_MODE | POS_SETPOINT_MODE)
    assert not session.telemetry().has(0x04)
    session.actuation(True)
    assert session.telemetry().has(0x04)
    session.land()
    assert session.disarm().has(0x04) is False
