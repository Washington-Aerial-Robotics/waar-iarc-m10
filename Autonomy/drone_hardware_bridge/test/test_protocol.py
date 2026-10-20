import math
import struct

import pytest

from drone_hardware_bridge.protocol import (
    COM_SET_TRAJSETPT, FLAG_ATTITUDE_VALID, FLAG_BATTERY_VALID,
    FLAG_POSITION_VALID, Packet, ReplyMatcher, TELEMETRY, build_packet,
    kaf_angular_velocity_to_ros,
    kaf_quaternion_to_ros_base, kaf_yaw_to_ros, pack_setpoint, parse_packet,
    ros_yaw_to_kaf, unpack_telemetry, yaw_from_quaternion,
)


def telemetry_payload(flags=FLAG_POSITION_VALID | FLAG_ATTITUDE_VALID):
    return TELEMETRY.pack(
        1, 0x0E, flags, 0, 7,
        1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
        0.0, 0.0, 0.0, 1.0,
        0.1, 0.2, 0.3, 55.0,
    )


def test_setpoint_wire_layout_is_exact_little_endian():
    payload = pack_setpoint(1, (2.0, 3.0, 4.0), 0.5, (0.1, 0.2, 0.3))
    assert len(payload) == 32
    assert struct.unpack("<I7f", payload)[0] == 1
    packet = parse_packet(build_packet("U", "P", COM_SET_TRAJSETPT, 9, payload))
    assert (packet.to_id, packet.from_id, packet.message_id) == (ord("U"), ord("P"), 9)
    assert packet.payload == payload


def test_setpoint_rejects_sequence_zero_and_nonfinite():
    with pytest.raises(ValueError):
        pack_setpoint(0, (0.0, 0.0, 0.0), 0.0)
    with pytest.raises(ValueError):
        pack_setpoint(1, (math.nan, 0.0, 0.0), 0.0)


def test_telemetry_exact_size_version_reserved_and_finiteness():
    decoded = unpack_telemetry(telemetry_payload())
    assert len(telemetry_payload()) == 64
    assert decoded.position == pytest.approx((1.0, 2.0, 3.0))
    bad = bytearray(telemetry_payload())
    bad[3] = 1
    with pytest.raises(ValueError):
        unpack_telemetry(bytes(bad))
    with pytest.raises(ValueError):
        unpack_telemetry(telemetry_payload()[:-1])


def test_valid_battery_flag_requires_a_real_percentage():
    payload = TELEMETRY.pack(
        1, 0x0E, FLAG_POSITION_VALID | FLAG_BATTERY_VALID, 0, 7,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 100.1,
    )
    with pytest.raises(ValueError, match="battery"):
        unpack_telemetry(payload)


def test_protocol_identifier_must_be_exactly_one_ascii_byte():
    assert parse_packet(build_packet("U", "P", 1, 2)).to_id == ord("U")
    for invalid in ("", "UP", "é", "Ué"):
        with pytest.raises(ValueError):
            build_packet(invalid, "P", 1, 2)


def test_reply_matcher_rejects_every_mismatched_envelope_field():
    matcher = ReplyMatcher("U", "P")
    good = Packet(ord("P"), ord("U"), 0x3C, 9, b"")
    assert matcher.matches(good, 9, 0x3C)
    assert not matcher.matches(Packet(ord("X"), ord("U"), 0x3C, 9, b""), 9, 0x3C)
    assert not matcher.matches(Packet(ord("P"), ord("X"), 0x3C, 9, b""), 9, 0x3C)
    assert not matcher.matches(Packet(ord("P"), ord("U"), 0x3D, 9, b""), 9, 0x3C)
    assert not matcher.matches(Packet(ord("P"), ord("U"), 0x3C, 8, b""), 9, 0x3C)


def test_kaf_ros_body_axes_and_yaw_round_trip():
    assert kaf_angular_velocity_to_ros((1.0, 2.0, 3.0)) == (2.0, -1.0, 3.0)
    # KAF yaw zero points its forward (+Y) north; ROS base +X therefore yaw +90 deg.
    q_ros = kaf_quaternion_to_ros_base((0.0, 0.0, 0.0, 1.0))
    assert yaw_from_quaternion(q_ros) == pytest.approx(math.pi / 2.0)
    for yaw in (-math.pi, -0.4, 0.0, 1.2, math.pi):
        assert math.sin(kaf_yaw_to_ros(ros_yaw_to_kaf(yaw))) == pytest.approx(math.sin(yaw))
        assert math.cos(kaf_yaw_to_ros(ros_yaw_to_kaf(yaw))) == pytest.approx(math.cos(yaw))
