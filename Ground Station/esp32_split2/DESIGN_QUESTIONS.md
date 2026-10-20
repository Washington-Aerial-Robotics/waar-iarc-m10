> **Kent compatibility update:** The app now targets local `origin/Firmware-Kent` commit `416ac12`. See [KENT_INTEGRATION.md](KENT_INTEGRATION.md) for current protocol and controls. Older protocol/control descriptions below are historical and are superseded by that document.

# Ground station design questions and answers

These answers describe the repository as reviewed on October 19, 2026. **Current behavior** is implemented code; **Recommendation** is proposed design work. The firmware flashed on a physical aircraft and competition compliance have not been verified.

## 1. Should the app send motor values, flight setpoints, or high-level commands?

**Answer:** Use flight setpoints for manual override and high-level behavioral commands for autonomous operation. Keep stabilization and motor mixing onboard. Direct motor commands should be restricted to explicit bench testing.

**Current behavior:** `DroneController` mixes throttle, pitch, roll, and yaw into four motor values and sends `comSetMotorCmd` (`0x69`). The repository firmware switches to motor-setpoint mode when it handles that command. The app is therefore currently a motor-command interface, rather than a completed supervisory autonomy interface.

**Next decision:** Agree with the firmware team on setpoint units, coordinate frame, limits, supported modes, and permitted mission commands. A packet builder for position commands exists, but that alone does not establish a working autonomous interface.

## 2. How does pilot override take control, and how is autonomy restored?

**Answer:** Use an explicit mode handover confirmed by the drone before enabling the corresponding controls.

**Current behavior:** ARM starts a local transmission timer. There is no completed app-level request/confirmation flow for taking control from autonomy or handing it back. Sending motor packets can change the repository firmware's mode, but that is not a confirmed handover shown to the operator.

**Recommendation:** Define authoritative onboard states such as autonomous, manual override, and failsafe. Define which command source owns control in each state, what happens during a pending transition, and how stale commands are rejected. Display the reported mode rather than inferring it from the local timer.

## 3. What exactly do ARM, DISARM, and KILL mean?

| Button | App behavior | Repository firmware evidence / limit |
| --- | --- | --- |
| ARM | Starts the 4 Hz motor-packet loop. | This is not a dedicated acknowledged arming procedure. Motor commands select motor-setpoint mode. |
| DISARM | Stops the local timer, clears inputs, sends zero motor values, then requests null mode (`0x68`, payload `0x00`). | The command handler accepts valid mode values and assigns the requested mode. Physical motor output and controller behavior still require verification. |
| KILL | Clears inputs, stops the loop, sends `0x05`, and locally requires reconnecting before further arming. | The Wi-Fi receive handler calls `ESP.restart()` for a matching KILL command addressed to this device. Restarting is not proof of a latched hardware motor shutdown. |

**Recommendation:** Establish and test actual motor behavior for each operation, including startup after KILL. Decide whether the firmware needs a latched emergency stop and independent stop path. Do not describe TCP transmission as proof that motors stopped.

## 4. How does the app confirm a command was accepted or executed?

**Answer:** It currently does not.

**Current behavior:** Packets include a message ID and the protocol defines success, failure, and acknowledgement types. However, the app does not assemble and correlate binary replies to pending commands. The displayed last voice command records recognition/dispatch, not execution.

**Recommendation:** Implement stream parsing, correlate replies using the agreed firmware response semantics, and show sent, accepted, rejected, or timed-out status. An accepted command is distinct from a completed action; use reported state to confirm execution where needed. Coordinate retry and duplicate-handling rules with firmware, especially for non-idempotent actions.

## 5. What happens when Wi-Fi drops, the app closes, or the phone locks?

**Answer:** A detected socket loss resets the app's controls and stops transmission. The app cannot guarantee aircraft behavior after communication is lost.

**Current behavior:** Reconnecting does not restart the loop; ARM is required again. There is no explicit app heartbeat deadline or completed lifecycle policy for backgrounding/locking. A silent network failure may not immediately trigger a socket error. Resetting local values does not send a stop to an unreachable drone.

**Recommendation:** Enforce command freshness and a defined failsafe onboard. Agree on behavior for manual and autonomous modes separately; losing manual input may require a different response from losing an optional monitoring link. Define app behavior on lock/background and verify it on both platforms. The aircraft must not rely on a final phone packet to enter its failsafe.

## 6. What transmission rate and timeout does firmware require?

**Answer:** The app sends at 4 Hz (one update every 250 ms). The required rate and control-link timeout have not been established by this review.

**Recommendation:** Select them with the flight-control team using measured delay, jitter, packet age, and the intended setpoint interface. Do not treat the earlier README's 50 Hz claim as a firmware requirement. Keep fast stabilization onboard; Dart timers and Wi-Fi do not provide deterministic flight-control timing.

**Verification:** Measure input-to-receipt latency and command age under load. Test missing updates, link interruptions, and recovery on a restrained bench setup before flight.

## 7. How are complete messages identified in TCP?

**Answer:** The app lacks a complete binary stream decoder.

**Current behavior:** Each message begins with destination ID, source ID, message type, and message ID. Payload size depends on message type; motor messages carry four little-endian float32 values. `TcpClient.packets` publishes socket chunks, which can contain partial messages or multiple messages. `tryParse` only checks for four header bytes and treats the rest as payload. Text logs separately split on newline; that is unsuitable for binary floats.

The repository firmware's Wi-Fi receive path also reads currently available bytes into a receive buffer. Its handling of split/coalesced traffic needs validation alongside the app.

**Recommendation:** Agree on one framing contract across both endpoints. Either define payload sizes for every supported type and implement buffered parsing, or coordinate a protocol revision with explicit framing/length fields. Test split headers, split payloads, combined messages, unknown types, and malformed data. Do not change framing on only one endpoint.

## 8. Which voice commands are required for competition?

**Answer:** The exact required vocabulary remains to be confirmed against the applicable rules and team mission design.

**Current behavior:** The app recognizes throttle up/down, forward/backward, lateral movement, rotation, and stop/center directions. These adjust manual inputs, not mission-level autonomous behavior. HOVER phrases are recognized but only report that altitude hold is unavailable. Voice arming, disarming, KILL, takeoff, landing, mission execution, and autonomy handover are not implemented.

**Recommendation:** Define a short explicit vocabulary with the autonomy team and map it to supported onboard behaviors. Distinguish stopping movement from stopping motors. Do not advertise hover until the onboard behavior exists. See [APP_GUIDE.md](APP_GUIDE.md#voice-commands) for every currently accepted phrase.

## 9. Does offline speech work on Android and iPhone?

**Answer:** The code requests on-device recognition (`onDevice: true`), but physical-device support has not been established.

**Current behavior:** Recognition uses `speech_to_text`, short button-triggered listening sessions, and exact matching of final phrases after normalization. There is no bundled independent speech-recognition model or continuous listening implementation.

**Verification:** Install the necessary language resources and permissions, connect to ESP32 Wi-Fi, disable cellular/other internet access, and test every supported phrase on each actual device. Include noise, silence, unsupported recognition services, and recognition errors. Confirm packet receipt as well as recognized text. Unsupported offline recognition should be reported clearly rather than assumed to work.

## 10. Which telemetry should the operator see?

**Answer:** Prioritize reported flight mode, control ownership, emergency-stop state, battery, connection health, and command confirmation. Add altitude/position and mission progress according to operator needs.

**Current behavior:** Remote readouts show local requested throttle/yaw/pitch/roll. Console can request position and Wi-Fi information, but structured replies are not yet presented as measured telemetry. There is no persistent session database.

**Recommendation:** Label requested and measured values separately. Show sample age and mark stale telemetry rather than displaying it as current. Timestamped event logs would support debugging; a database can wait until searchable history is needed.

## 11. Which interactions do the competition rules permit?

**Answer:** Compliance is unresolved. The intended team role is emergency stop, pilot override, and voice input, but that intention does not establish what is permitted during a scored run.

The exact linked Mission 10 revision 3.1.2 was not successfully retrieved in the prior discussion. This document does not assert that a phone-based recognizer, manual override, offboard processing, or any specific operator command is approved.

**Next decision:** Review the exact applicable rules and clarifications with the team. Record permitted setup, qualification, mission, and emergency interactions separately, including where speech processing may run. Obtain organizer clarification for ambiguity before finalizing the architecture.

## Recommended order of work

1. Agree on the command boundary and verify the deployed firmware, particularly KILL/restart behavior.
2. Define onboard control ownership, emergency stop, and link-loss behavior.
3. Implement matching stream parsing, response correlation, and reported mode/telemetry.
4. Integrate setpoints and high-level voice commands with onboard autonomy.
5. Validate offline operation on Android and iPhone and verify permitted competition use.

## Code references

- [Drone controller](lib/controllers/drone_controller.dart)
- [Voice recognition and phrase matching](lib/controllers/voice_command_controller.dart)
- [TCP transport](lib/services/tcp_client.dart)
- [App protocol](lib/protocol/drone_protocol.dart)
- [Firmware protocol definitions](../../ESP32/KAF_Drone/src/communication.h)
- [Firmware command handlers](../../ESP32/KAF_Drone/src/command_task.cpp)
- [Firmware Wi-Fi receive / KILL handling](../../ESP32/KAF_Drone/src/firmware.cpp)
- [High-level app guide](APP_GUIDE.md)
