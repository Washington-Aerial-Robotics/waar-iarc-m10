# Flutter Drone Controller GUI

## Overview

This repository implements a **cross-platform Flutter-based drone control interface** for real-time communication with an ESP32 flight controller over TCP/Wi-Fi.

The application provides:

* live TCP console communication (send raw test/debug packets, watch raw firmware output)
* dual joystick flight controls
* ARM / DISARM / KILL safety controls
* voice commands (basic movement, hover, stop)
* responsive portrait + landscape layouts

---

## Getting Started

### Prerequisites

* Flutter SDK, **stable channel** (this project pins `sdk: ^3.9.2` in `pubspec.yaml` — run `flutter --version` and `flutter doctor` to confirm your install satisfies it).
* A phone (Android or iOS) or desktop target to run on. Voice control (`speech_to_text`) is the one feature that's only verified on mobile — don't assume it works on Windows/macOS/Linux builds.
* The drone powered on, with its flight firmware running (`ESP32/KAF_Drone`).

### Install & run

```bash
cd "Ground Station/esp32_split2"
flutter pub get
flutter devices        # confirm your target device/emulator is visible
flutter run            # or pick a device explicitly: flutter run -d <device-id>
```

You can also launch it from VS Code using the configurations in `.vscode/launch.json` (`esp32_split2`, plus profile/release variants).

Platform permissions the app needs are already declared:

* **Android** (`android/app/src/main/AndroidManifest.xml`): `RECORD_AUDIO` (voice control) and `INTERNET` (TCP socket to the drone).
* **iOS** (`ios/Runner/Info.plist`): `NSMicrophoneUsageDescription` and `NSSpeechRecognitionUsageDescription`.

If you add other platforms later, double-check they have equivalent permissions — network/mic access is not implicitly granted everywhere.

---

## Connecting to the drone

The app does **not** join Wi-Fi networks itself — it only opens a TCP socket to an IP/port you give it. You have to join the drone's Wi-Fi network at the OS level (phone Settings → Wi-Fi) *before* connecting from inside the app.

By default, the drone boots as its own open Wi-Fi access point:

* **SSID:** `KAF_Quadcopter_Drone`
* **Password:** none (open network)
* **Drone IP once connected:** `192.168.1.240`
* **TCP port:** `70` (hardcoded in firmware — `WiFiServer server = WiFiServer(70)` in `periph_wifi.cpp`)

> **Heads up:** the Console tab's IP/port fields currently default to `172.20.10.8` / `80` — leftovers from a previous test setup, not the drone's actual defaults. For a stock drone in its default AP mode, clear those fields and enter `192.168.1.240` and `70` instead. If your drone has been reconfigured to join an existing network (STA mode) or a different port, use whatever IP/port it was actually assigned there.

The drone's firmware only accepts **one TCP client at a time** — if another ground station (or a stale connection) is already attached, a new connection attempt will hang or fail until that one disconnects.

### Step by step

1. Power on the drone. It starts broadcasting `KAF_Quadcopter_Drone`.
2. On your phone, join that Wi-Fi network in system settings.
3. Open the app → **Console** tab.
4. Set **Target Drone ID** (single character, default `A` — must match the drone's configured ID).
5. Set **Drone IP** to `192.168.1.240` and **Port** to `70` (see note above).
6. Tap **Connect to drone**. You should see `Connecting…` then `Connected` in the log, and the connection status icon turn green.
7. Use **Send PING** to sanity-check the link — you should see a logged reply.

---

## Using the app

### Console tab

Low-level debug view. Connect/disconnect, change target drone ID, and fire off individual test packets:

* **Send PING** — round-trip check that the link is alive.
* **Get IP** — asks the drone to report its Wi-Fi info.
* **Get Pos** — requests the drone's current position/flight status.
* **Test Control** — sends a single motor-command packet with all four motors at 0.5, independent of the joysticks (useful for a quick bench test with props off).

All traffic is logged as raw hex in **Drone Printout**.

### Remote tab (primary flight interface)

* **Left stick** — throttle (up/down) / yaw (left/right).
* **Right stick** — pitch (forward/back) / roll (left/right).
* **ARM** — starts the app's control loop, which sends motor commands to the drone 4 times per second (every 250ms) based on current stick position.
* **DISARM** — zeroes all stick values and stops the control loop.
* **KILL** — immediately zeroes controls, stops the loop, *and* sends a dedicated kill packet to the drone's firmware (this is the one button that reaches the drone even if the local loop isn't running).
* Live readout of throttle/yaw/pitch/roll is shown in the HUD — note this reflects the app's own stick state, not telemetry reported back by the drone.

**Safety notes:**
* You must be connected *and* armed before sticks or voice commands do anything — the controls are disabled otherwise.
* Once "reconnect after KILL" happens (i.e. you've hit KILL), you must disconnect and reconnect before arming again.

### Voice control

Available on the Remote tab once armed. Tap the mic button and speak one phrase per command (recognized as a single short utterance, not continuous dictation):

| Say | Effect |
|---|---|
| "up" / "go up" / "move up" / "ascend" | increases throttle a step |
| "down" / "go down" / "move down" / "descend" | decreases throttle a step |
| "hover" / "maintain altitude" / "hold altitude" | sets throttle to a fixed hover value, centers pitch/roll/yaw |
| "stop" / "hold" / "stop moving" / "center controls" | centers pitch/roll/yaw, keeps current throttle |
| "forward" / "move forward" / "go forward" | pitches forward briefly, then auto-recenters |
| "backward" / "back" / "move backward" / "go backward" | pitches backward briefly, then auto-recenters |
| "move left" / "slide left" / "strafe left" | rolls left briefly, then auto-recenters |
| "move right" / "slide right" / "strafe right" | rolls right briefly, then auto-recenters |
| "rotate left" / "turn left" / "yaw left" | yaws left briefly, then auto-recenters |
| "rotate right" / "turn right" / "yaw right" | yaws right briefly, then auto-recenters |

Directional commands (forward/back/left/right/rotate) automatically recenter after ~750ms; throttle commands (up/down/hover) persist until changed again.

---

## Architecture

```text
Flutter UI Layer
│
├── Console Screen      — connect/disconnect, raw test packets, log view
├── Remote Control Screen — joysticks, ARM/DISARM/KILL, voice panel
│
▼
Controller Layer
│
├── DroneController        — connection state, stick state, safety commands, 4Hz TX loop
└── VoiceCommandController — wraps speech_to_text, maps phrases to DroneController calls
│
▼
Protocol Layer
│
└── drone_protocol.dart — builds/parses the binary packet format, message-type constants
│
▼
Networking Layer
│
└── TcpClient — raw dart:io Socket, TX/RX byte logging
│
▼
ESP32 firmware (../../ESP32/KAF_Drone)
```

State is shared via `provider`: `main.dart` sets up `TcpClient` → `DroneController` → `VoiceCommandController`, in that dependency order.

---

## Networking protocol

Communication is a **binary packet protocol** over a raw TCP socket (not line-delimited text).

### Packet layout

```text
byte 0: toId        — target device ID (single byte, e.g. 'A' = 0x41)
byte 1: fromId       — sender device ID (app is always 0x47 'G')
byte 2: messageType  — command code (see drone_protocol.dart / ESP32 communication.h)
byte 3: messageId    — rolling sequence number
bytes 4+: payload    — command-specific (e.g. 4 little-endian floats for motor commands)
```

Message-type constants live in `lib/protocol/drone_protocol.dart` and must match `ESP32/KAF_Drone/src/core/communication.h` on the firmware side exactly — they're maintained by hand on both sides, so re-check them against the firmware header if you add or change a command type.

### Motor control packets

Sent by the armed control loop at **4Hz (every 250ms)**: stick values are curve-shaped, mixed into 4 motor values via a standard X-quad mixer, and packed as 4 little-endian 32-bit floats (16-byte payload) behind the standard 4-byte header.

---

## Current file structure

```text
lib/
│
├── main.dart
│
├── controllers/
│   ├── drone_controller.dart
│   └── voice_command_controller.dart
│
├── protocol/
│   └── drone_protocol.dart
│
├── services/
│   └── tcp_client.dart
│
├── screens/
│   ├── drone_tcp_console.dart
│   ├── drone_remote_control.dart
│   └── beta_remote_preview.dart   (UI sandbox — not wired into the app's navigation)
│
└── widgets/
    ├── connect_panel.dart
    ├── log_view.dart
    └── voice_control_panel.dart
```

---

## Known gaps

* No telemetry is actually parsed or displayed from the drone yet — the "live" readout on the Remote tab is the app's own stick state, not anything reported back over the wire.
* ARM/DISARM are local-only; no packet is sent to the drone for either (KILL is the exception — it does send a real packet).
* No automated tests yet.

---

## Tech stack

* Flutter / Dart
* Provider (state management)
* `speech_to_text` (voice recognition)
* Raw TCP sockets (`dart:io`)
* ESP32 firmware backend (`ESP32/KAF_Drone`)

---

## Purpose

This project is being developed as a **real-time drone control GUI prototype** to be used by Washington Aerial Robotics.
