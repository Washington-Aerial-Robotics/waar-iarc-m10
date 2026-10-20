> **Kent compatibility update:** The app now targets local `origin/Firmware-Kent` commit `416ac12`. See [KENT_INTEGRATION.md](KENT_INTEGRATION.md) for current protocol and controls. Older protocol/control descriptions below are historical and are superseded by that document.

# Ground station app guide

See [DESIGN_QUESTIONS.md](DESIGN_QUESTIONS.md) for answers about architecture, override, safety commands, telemetry, and competition readiness.

This Flutter app connects an Android phone or iPhone to an ESP32 over local Wi-Fi and TCP. Its current implementation provides development controls, motor commands, and short voice-command sessions. Internet is not required for the TCP connection. Offline speech availability depends on the device and installed language support.

## Where to find things

| File | Responsibility |
| --- | --- |
| `lib/main.dart` | Starts the app, creates shared TCP/drone/voice controllers with Provider, and displays Console and Remote tabs. |
| `lib/screens/drone_tcp_console.dart` | Connection settings, ping, IP/position requests, test motor commands, and received text logs. |
| `lib/screens/drone_remote_control.dart` | Portrait/landscape joystick layout, local control readouts, ARM/DISARM/KILL buttons, and voice panel. |
| `lib/screens/beta_remote_preview.dart` | Experimental local controller preview; currently absent from the main navigation. |
| `lib/controllers/drone_controller.dart` | Connection/control state, target drone ID, motor mixing, periodic transmission, and voice movement actions. |
| `lib/controllers/voice_command_controller.dart` | Initializes speech recognition, listens for completed phrases, matches accepted commands, and calls drone actions. See `_executeCommand` for the phrase list. |
| `lib/services/tcp_client.dart` | Opens/closes TCP sockets, sends bytes, publishes received byte chunks and newline-delimited text, and reports socket connection changes. |
| `lib/protocol/drone_protocol.dart` | Command constants and binary packet construction; contains a basic packet parser. |
| `lib/widgets/connect_panel.dart` | IP, port, target-ID inputs, and connection button. |
| `lib/widgets/log_view.dart` | Displays log lines supplied by its parent. |
| `lib/widgets/voice_control_panel.dart` | Speech button, recognition state, recognized text, last command, and errors. |
| `test/control_safety_test.dart` | Regression checks for link-loss reset, DISARM packets, disabled hover, and KILL ID. |
| `test/remote_layout_test.dart` | Landscape layout checks with safe areas and bottom navigation at two phone sizes. |
| `android/app/src/main/AndroidManifest.xml` | Android networking/microphone permissions and speech-service visibility. |
| `ios/Runner/Info.plist` | iPhone local-network, microphone, and speech permission descriptions. |
| `ios/Podfile` and `ios/Runner.xcodeproj/project.pbxproj` | iOS build settings, including the iOS 15 minimum. |

## Connection and command flow

1. Power on the ESP32 and join its Wi-Fi, or put both devices on the same reachable network, according to the firmware configuration.
2. Open Console and enter the actual ESP32 IP, TCP server port, and target device ID. Defaults are editable examples, not discovered settings.
3. Connect. Establishing TCP does not start the motor transmission loop or prove firmware compatibility.
4. Use Remote for joystick input or tap ARM before issuing movement voice commands.
5. Inputs flow from the UI to `DroneController`, then through `DronePacketBuilder` and `TcpClient` to the ESP32.

The current motor loop runs every 250 ms (4 Hz). It applies square-root input curves and mixes throttle/pitch/roll/yaw into four motor values. These are direct motor setpoints, not onboard autonomous navigation instructions. Validate mixing, rate, and the flashed firmware before operating hardware.

## Buttons and controls

| Control | Current behavior |
| --- | --- |
| ARM | Starts the local motor transmission loop. Does not wait for an onboard armed-state acknowledgement. |
| DISARM | Clears local inputs, stops the loop, sends four zero motor values, then requests null control mode if connected. Delivery/execution is unconfirmed. |
| KILL | Stops local transmission, clears inputs, and sends the firmware KILL command. Local controls require reconnecting afterward. No acknowledgement is checked. |
| Left joystick | Sets throttle and yaw. Releasing/cancelling centers yaw and sets throttle to zero. |
| Right joystick | Sets pitch and roll. Releasing/cancelling centers both. |
| Send PING | Sends a ping request to the selected drone ID. |
| Get IP / Get Pos | Sends a Wi-Fi-address or position request. Structured responses are not yet decoded into a telemetry dashboard. |
| Test Control | Sends four motor values of 0.5 immediately, independently of the Remote ARM loop. Treat this as a live motor command; test with propellers removed. |

Remote numeric readouts show local input values; they are not measured drone telemetry. When the socket reports a disconnect, controls reset and the timer stops. Reconnecting does not restart that timer. A silent outage can take time to be detected; there is no explicit heartbeat timeout yet.

## Voice commands

Connect, ARM, and tap **Speak command**. Recognition requests on-device processing. Listening is requested for six seconds with a two-second pause limit; the speech service can stop earlier. Only final recognition results execute commands. Matching ignores case and punctuation, but otherwise requires an exact accepted phrase; extra words do not match.

| Action | Accepted phrases |
| --- | --- |
| Increase throttle | `up`, `go up`, `move up`, `ascend` |
| Decrease throttle | `down`, `go down`, `move down`, `descend` |
| Forward | `forward`, `move forward`, `go forward` |
| Backward | `back`, `backward`, `move backward`, `go backward` |
| Move left | `move left`, `slide left`, `strafe left` |
| Move right | `move right`, `slide right`, `strafe right` |
| Rotate left | `rotate left`, `turn left`, `yaw left` |
| Rotate right | `rotate right`, `turn right`, `yaw right` |
| Center directional inputs | `stop`, `hold`, `center controls`, `stop moving` |
| Hover (unavailable) | `hover`, `maintain altitude`, `hold altitude` are recognized, but only log that onboard altitude hold is not implemented. |

Up/down changes throttle by 0.08 per command. Directional commands set an input of magnitude 0.25 and automatically center after 750 ms. STOP centers pitch, roll, and yaw while preserving throttle; it does not stop motors. These values pass through the motor input curves before transmission.

Voice ARM, DISARM, KILL, autonomous/manual mode switching, takeoff, landing, and mission-level navigation are not implemented. The displayed last command records recognition/dispatch, not drone execution. The panel currently still advertises hover despite its disabled action.

## Protocol and current limits

Packets start with four bytes: destination ID, source ID, message type, and message ID. Motor payloads contain four little-endian 32-bit floats. Current command IDs were aligned with `ESP32/KAF_Drone/src/communication.h` elsewhere in this repository; the firmware actually flashed on the ESP32 must match.

The TCP byte stream is exposed as arbitrary received chunks. The simple packet parser does not assemble a full framed stream or validate payload lengths. The app does not yet correlate acknowledgements, prove command execution, provide confirmed pilot/autonomy handover, or implement onboard failsafes. Those require coordinated firmware work.

Logs are held in memory; there is no database, persistent flight-session history, or export feature. This app guide describes the code, not verified competition-rule compliance or completed hardware qualification.

## Development checks and field launch

Run `flutter analyze` and `flutter test` for static checks and regression tests. These do not replace physical-device testing.

Use `flutter run` during development. For field use, install signed release builds for Android/iOS beforehand, then launch from the phone icon, join the ESP32 network, and connect in Console. No development laptop is needed to launch an installed app. Verify offline speech separately on each platform.
