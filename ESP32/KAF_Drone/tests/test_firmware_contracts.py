"""Host-runnable, hardware-honest contract tests for KAF_Drone firmware.

These tests deliberately do not claim to simulate sensors, FreeRTOS, Wi-Fi, or motors. They check the
wire ABI and safety-critical source invariants that can regress without hardware. Run on Ubuntu with:

    python3 -m unittest discover -s ESP32/KAF_Drone/tests -p 'test_*.py' -v

The separate on-drone checklist in FULL_STACK_TEST_PLAN.md is still mandatory before fitting propellers.
"""

from __future__ import annotations

import math
import re
import struct
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMMANDER_CPP = (ROOT / "src/auxilary/commander.cpp").read_text(encoding="utf-8")
COMMANDER_H = (ROOT / "src/auxilary/commander.h").read_text(encoding="utf-8")
COMMUNICATION_CPP = (ROOT / "src/core/communication.cpp").read_text(encoding="utf-8")
COMMUNICATION_H = (ROOT / "src/core/communication.h").read_text(encoding="utf-8")
FLIGHT_CPP = (ROOT / "src/core/flight.cpp").read_text(encoding="utf-8")
ESTIMATION_CPP = (ROOT / "src/auxilary/estimation.cpp").read_text(encoding="utf-8")
GPS_CPP = (ROOT / "src/firmware/periph_samm10q.cpp").read_text(encoding="utf-8")
WIFI_CPP = (ROOT / "src/firmware/periph_wifi.cpp").read_text(encoding="utf-8")
PID_TUNER_CPP = (ROOT / "src/auxilary/pid_tuner.cpp").read_text(encoding="utf-8")
KAF_DRONE_CPP = (ROOT / "src/core/kaf_drone.cpp").read_text(encoding="utf-8")


def autonomy_frame_length(buffer: bytes) -> int | None:
    """Reference model of the fixed-frame Wi-Fi accumulator (None means legacy/unknown)."""
    if len(buffer) < 4:
        return 4
    message_type = buffer[2]
    if message_type in (0x64, 0x65):
        return 4
    if message_type in (0x4A, 0x5B):
        return 5
    if message_type == 0x5D:
        return 36
    if message_type == 0x4C:
        return 6 if len(buffer) < 6 else 6 + 4 * buffer[5]
    return None


class WireAbiTests(unittest.TestCase):
    def test_autonomy_message_ids_are_stable(self) -> None:
        self.assertRegex(COMMUNICATION_H, r"COM_SET_TRAJSETPT\s+\(\s*COM_CMD\s*\|\s*29\s*\)")
        self.assertRegex(COMMUNICATION_H, r"COM_SET_GPSORIGIN\s+\(\s*COM_CMD\s*\|\s*36\s*\)")
        self.assertRegex(COMMUNICATION_H, r"COM_REQUEST_TELEMETRY\s+\(\s*COM_CMD\s*\|\s*37\s*\)")
        self.assertRegex(COMMUNICATION_H, r"COM_REPLY_TELEMETRY\s+\(\s*37\s*\)")

    def test_pi_setpoint_layout_is_exactly_32_little_endian_bytes(self) -> None:
        payload = struct.pack("<I7f", 0x12345678, 1, 2, 3, 4, 5, 6, 7)
        self.assertEqual(len(payload), 32)
        self.assertEqual(payload[:4], b"xV4\x12")
        self.assertIn("static_assert( sizeof( trajsetpoint ) == 32", COMMANDER_H)
        self.assertIn("uint32_t sequence", COMMANDER_H)

    def test_telemetry_layout_is_exactly_64_bytes(self) -> None:
        fmt = "<BBBBI3f3f4f3ff"
        values = (1, 0x0E, 0x3F, 0, 7, *range(3), *range(3), 0, 0, 0, 1, *range(3), 91.5)
        payload = struct.pack(fmt, *values)
        self.assertEqual(struct.calcsize(fmt), 64)
        self.assertEqual(len(payload), 64)
        self.assertIn("static_assert( sizeof( autonomy_telemetry ) == 64", COMMANDER_H)


class SafetySourceContractTests(unittest.TestCase):
    @staticmethod
    def _between(start: str, end: str) -> str:
        begin = COMMANDER_CPP.index(start)
        finish = COMMANDER_CPP.index(end, begin)
        return COMMANDER_CPP[begin:finish]

    def test_rejected_custom_commands_cannot_be_processed(self) -> None:
        self.assertIn("validatedReplyHandlers", COMMUNICATION_CPP)
        self.assertIn("*response = NULLPTR", COMMANDER_CPP)
        self.assertRegex(
            COMMUNICATION_CPP,
            r"rxheader\.messageType\s*=\s*messageType\s*==\s*COM_FAILURE\s*\?\s*COM_INVALID",
        )

    def test_setters_reply_success_and_telemetry_uses_dedicated_reply_type(self) -> None:
        # Validated setters inherit COM_SUCCESS. Only telemetry opts into its data-bearing reply type.
        setpoint = self._between("com_receiveValidatedMessage( COM_SET_TRAJSETPT", "//Origin latching")
        origin = self._between("com_receiveValidatedMessage( COM_SET_GPSORIGIN", "com_receiveValidatedMessage( COM_REQUEST_TELEMETRY")
        telemetry = self._between("com_receiveValidatedMessage( COM_REQUEST_TELEMETRY", 'return { "commander"')
        self.assertNotIn("COM_REPLY_TELEMETRY", setpoint)
        self.assertNotIn("COM_REPLY_TELEMETRY", origin)
        self.assertIn("COM_REPLY_TELEMETRY", telemetry)
        self.assertIn("successReplyType = COM_SUCCESS", COMMUNICATION_H)
        self.assertIn("coms.validatedSuccessTypes[handlerIndex]", COMMUNICATION_CPP)

    def test_origin_command_is_pi_only_disarmed_and_resets_stream_session(self) -> None:
        block = self._between(
            "com_receiveValidatedMessage( COM_SET_GPSORIGIN",
            "com_receiveValidatedMessage( COM_REQUEST_TELEMETRY",
        )
        for token in (
            "header.fromID != PI_CONTROLLER_ID",
            "kafenv.info.actuation",
            "const bool originLatched = gps_setOrigin()",
            "commander_resetPiSequence()",
        ):
            self.assertIn(token, block)

    def test_stream_acceptance_is_pi_only_finite_fresh_and_monotonic(self) -> None:
        block = re.search(r"static bool piSetpointValid\(.*?\n}\n", COMMANDER_CPP, re.DOTALL)
        self.assertIsNotNone(block)
        text = block.group(0)
        self.assertIn("header.fromID != PI_CONTROLLER_ID", text)
        self.assertIn("!estimation_positionValid()", text)
        self.assertIn("setpt->sequence <= kafenv.cmd.setpointSeq", text)
        for field in ("x", "y", "z", "yaw", "vx", "vy", "vz"):
            self.assertIn(f"isfinite( setpt->{field} )", text)

    def test_only_pi_stream_uses_500ms_watchdog(self) -> None:
        self.assertIn("const bool setpointStale = commander_piStreamActive()", COMMANDER_CPP)
        self.assertNotIn("kafenv.cmd.setpointMillis == 0 ||", COMMANDER_CPP)

    def test_circle_keeps_timer_at_index_zero(self) -> None:
        implementation = self._between("static bool setTrajectories", "bool commander_setTrajectories")
        circle_start = implementation.index("case FLIGHTPATH_CIRCLE")
        text = implementation[circle_start:implementation.index("break;", circle_start)]
        self.assertIn("setpoints[ 9] = u2a", text)
        self.assertNotRegex(text, r"setpoints\[\s*0\s*\]\s*=")

    def test_pi_can_request_only_land_and_trajectory_never_auto_arms(self) -> None:
        handler = self._between("com_receiveValidatedMessage( COM_SET_TRAJECTORY", "com_receiveMessage( COM_SET_TRAJCONFIG")
        self.assertIn("header.fromID == PI_CONTROLLER_ID && trajectory == FLIGHTPATH_LAND", handler)
        implementation = self._between("static bool setTrajectories", "bool commander_setTrajectories")
        self.assertIn("const bool wasActuating = kafenv.info.actuation", implementation)
        self.assertIn("kafenv.info.actuation = wasActuating", implementation)
        self.assertNotIn("kafenv.info.actuation = true", implementation)
        self.assertIn("CMD_DESCENT_MODE | ( kafenv.info.flightMode & DEFAULT_MODES_MASK )", handler)
        self.assertIn("!kafenv.info.actuation ) return", handler)

    def test_sender_authorization_is_fail_closed(self) -> None:
        validator = self._between("bool commander_validateFlightModeCommand", "void commander_acceptFlightModeCommand")
        self.assertIn("mode != ( CMD_NOMINAL_MODE | POS_SETPOINT_MODE ) || length != 0", validator)
        self.assertIn("sender != GROUND_STATION_ID", validator)
        arming = self._between("bool commander_canArm", "static bool piSetpointValid")
        self.assertIn("sender == PI_CONTROLLER_ID", arming)
        self.assertIn("sender != GROUND_STATION_ID", arming)
        self.assertIn("rxheader.fromID == GROUND_STATION_ID", COMMUNICATION_CPP)

    def test_unknown_senders_cannot_mutate_legacy_state_or_actuation(self) -> None:
        start = COMMUNICATION_CPP.index("static bool commandSenderAuthorized")
        end = COMMUNICATION_CPP.index("static unsigned short fixedMutationSize", start)
        allowlist = COMMUNICATION_CPP[start:end]
        self.assertIn("COM_SET_ACTUATION", allowlist)
        self.assertIn("sender == GROUND_STATION_ID || sender == PI_CONTROLLER_ID", allowlist)
        for command in (
            "COM_SET_SENDMSG", "COM_SET_INFO", "COM_SET_STEST", "COM_SET_SETPT", "COM_SET_MOTORS",
            "COM_SET_CALIB", "COM_SET_KAFENV", "COM_SET_MEMORY", "COM_SET_INVOKEFUNC", "COM_SET_WIFI",
        ):
            self.assertIn(command, allowlist)
        self.assertRegex(allowlist, r"case COM_SET_KILL\s*:\s*\n\s*return true")
        actuation = COMMUNICATION_CPP[
            COMMUNICATION_CPP.index("case COM_SET_ACTUATION"):
            COMMUNICATION_CPP.index("case COM_REQUEST_DEVICES")
        ]
        self.assertIn("rx->actuation == 0 || requestedActuation", actuation)
        self.assertIn("!commandSenderAuthorized", COMMUNICATION_CPP)

    def test_builtin_mutations_require_exact_payload_sizes(self) -> None:
        self.assertIn("sizeof( rx->flightmode.h ) + valuesSize == *messageSize", COMMUNICATION_CPP)
        self.assertIn("sizeof( rx->setpoint.length ) + valuesSize == *messageSize", COMMUNICATION_CPP)
        self.assertIn("rx->memtransfer.h.length != contentSize", COMMUNICATION_CPP)
        self.assertIn("*messageSize == fixedMutationSize( *messageType )", COMMUNICATION_CPP)
        self.assertIn("coms.contentSizes[COM_SET_KAFENV]", COMMUNICATION_CPP)

    def test_physical_pid_tuner_exit_never_rearms(self) -> None:
        start = PID_TUNER_CPP.index("case TUNING_MODE_PHYS")
        end = PID_TUNER_CPP.index("case TUNING_MODE_EXIT", start)
        block = PID_TUNER_CPP[start:end]
        self.assertIn("kafenv.info.actuation = false", block)
        self.assertNotIn("kafenv.info.actuation = true", block)

    def test_wifi_accumulates_without_available_byte_truncation(self) -> None:
        self.assertIn("unsigned short receiveLength", WIFI_CPP)
        self.assertIn("wifiReadUpTo( sizeof( packet_header ) )", WIFI_CPP)
        self.assertIn("wifiReadUpTo( frameLength )", WIFI_CPP)
        self.assertIn("wifi.receiveLength = 0", WIFI_CPP)
        self.assertNotIn("(unsigned char)wifi.client.available()", WIFI_CPP)

    def test_reference_framer_handles_fragmentation_and_coalescing(self) -> None:
        setpoint = bytes((ord("U"), ord("P"), 0x5D, 1)) + struct.pack("<I7f", 1, *([0.0] * 7))
        self.assertEqual(len(setpoint), 36)
        accumulated = b""
        for chunk in (setpoint[:1], setpoint[1:4], setpoint[4:17], setpoint[17:35]):
            accumulated += chunk
            self.assertGreater(autonomy_frame_length(accumulated), len(accumulated))
        accumulated += setpoint[35:]
        self.assertEqual(autonomy_frame_length(accumulated), len(accumulated))

        telemetry = bytes((ord("U"), ord("P"), 0x65, 2))
        coalesced = setpoint + telemetry
        first_length = autonomy_frame_length(coalesced)
        self.assertEqual(first_length, 36)
        self.assertEqual(autonomy_frame_length(coalesced[first_length:]), 4)

    def test_gps_callbacks_are_not_reversed(self) -> None:
        self.assertIn(
            '{ "samm10q", 0, sizeof( sam ), &sam, &peripheral_samm10qInit, &peripheral_samm10qLoop }',
            GPS_CPP,
        )

    def test_soft_ap_gateway_is_on_same_subnet(self) -> None:
        self.assertIn(
            "WiFi.softAPConfig( IPAddress( 192, 168, 1, 240 ), IPAddress( 192, 168, 1, 240 )",
            WIFI_CPP,
        )
        self.assertNotIn("IPAddress( 192, 168, 0, 1 )", WIFI_CPP)

    def test_quaternion_conversion_normalizes_and_has_safe_identity_fallback(self) -> None:
        self.assertIn("bool flight_rotationQuaternion", FLIGHT_CPP)
        self.assertIn("quaternion[3] = 1", FLIGHT_CPP)
        self.assertRegex(FLIGHT_CPP, r"sqrtf\( x \* x \+ y \* y \+ z \* z \+ w \* w \)")
        self.assertIn("row0Norm < 0.9F", FLIGHT_CPP)
        self.assertIn("determinant < 0.9F", FLIGHT_CPP)

    def test_origin_rebases_fresh_baro_and_never_reuses_old_gps_coordinates(self) -> None:
        self.assertIn("void estimation_latchLocalOrigin()", ESTIMATION_CPP)
        latch_start = ESTIMATION_CPP.index("void estimation_latchLocalOrigin()")
        latch_end = ESTIMATION_CPP.index("//DO NOT MODIFY ANYTHING BELOW", latch_start)
        latch = ESTIMATION_CPP[latch_start:latch_end]
        self.assertIn("estimation.lastGpsXY = { 0, 0, 0 }", latch)
        self.assertIn("lastUpdate != 0", latch)
        self.assertIn("estimation.baroOriginZ = freshBarometer ? estimation.lastBaroZ : 0", latch)
        self.assertIn("estimation.lastBaroZ - estimation.baroOriginZ", ESTIMATION_CPP)
        self.assertIn("estimation.baroOriginSet && estimation.lastBaroZMillis != 0", ESTIMATION_CPP)
        origin = self._between(
            "com_receiveValidatedMessage( COM_SET_GPSORIGIN",
            "com_receiveValidatedMessage( COM_REQUEST_TELEMETRY",
        )
        self.assertIn("estimation_latchLocalOrigin()", origin)

    def test_pi_arm_requires_measured_control_calibration(self) -> None:
        self.assertIn("kafenv.cal.hoverThrust = 0", KAF_DRONE_CPP)
        arming = self._between("bool commander_canArm", "static bool controllerGainsValid")
        self.assertIn("commander_controlCalibrated()", arming)
        calibration = self._between("bool commander_controlCalibrated", "bool commander_batteryValid")
        self.assertIn("kafenv.cal.hoverThrust > CONTROL_HOVER_THRUST_MIN", calibration)
        self.assertIn("kafenv.cal.hoverThrust < CONTROL_HOVER_THRUST_MAX", calibration)
        self.assertIn("controllerGainsValid( kafenv.cal.xpid )", calibration)
        self.assertIn("controllerGainsValid( kafenv.cal.wpid[i] )", calibration)
        self.assertIn("AUTONOMY_FLAG_CONTROL_CALIBRATED", COMMANDER_H)
        self.assertIn("AUTONOMY_FLAG_CONTROL_CALIBRATED", COMMANDER_CPP)

    def test_battery_is_explicitly_invalid_until_a_real_measurement_exists(self) -> None:
        self.assertIn("AUTONOMY_FLAG_BATTERY_VALID", COMMANDER_H)
        battery_start = COMMANDER_CPP.index("bool commander_batteryValid()")
        battery_end = COMMANDER_CPP.index("static bool piSetpointValid", battery_start)
        battery = COMMANDER_CPP[battery_start:battery_end]
        self.assertRegex(battery, r"return false\s*;")
        self.assertIn("commander_batteryValid() ? AUTONOMY_FLAG_BATTERY_VALID : 0", COMMANDER_CPP)
        self.assertIn("const bool lowBattery = commander_batteryValid()", COMMANDER_CPP)

    def test_reference_quaternion_math_is_normalized(self) -> None:
        # Independent host-side sanity check for the expected ROS xyzw convention on a +90 deg yaw.
        yaw = math.pi / 2
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        self.assertAlmostEqual(sum(component * component for component in q), 1.0, places=7)
        self.assertGreater(q[2], 0)


if __name__ == "__main__":
    unittest.main()
