# Flutter Drone Controller GUI

## Overview

This repository implements a **cross-platform Flutter-based drone control interface** for real-time communication with an ESP32 flight system over TCP/Wi-Fi.

The application is designed to provide:

* **live TCP console communication**
* **dual joystick flight controls**
* **ARM / DISARM / KILL safety controls**
* **real-time telemetry display**
* **responsive portrait + landscape layouts**
* **modular networking + controller architecture**

The system is intended for **rapid prototyping, simulation, and live drone control testing**.

---

## Core Features

### Console Mode

Provides direct TCP communication with the drone firmware.

Features:

* connect / disconnect to ESP32 over Wi-Fi
* send test packets
* send ping packets
* monitor incoming logs
* view raw firmware output

---

### Remote Control Mode

Primary pilot interface.

Includes:

* **left joystick**

  * throttle
  * yaw

* **right joystick**

  * pitch
  * roll

* **HUD control panel**

  * ARM
  * DISARM
  * KILL

* **live telemetry readout**

  * throttle
  * yaw
  * pitch
  * roll

---

### Beta Sandbox Mode

Experimental UI testing environment.

Used for:

* joystick tuning
* layout testing
* control experiments
* interface prototyping

This mode runs independently from live drone connection.

---

## Architecture

The application follows a **layered modular architecture**.

```text
Flutter UI Layer
│
├── Console Screen
├── Remote Control Screen
├── Beta Preview Screen
│
▼
Controller Layer
│
├── DroneController
│   ├── control state
│   ├── safety commands
│   ├── periodic TX loop
│   └── telemetry state
│
▼
Networking Layer
│
└── TcpClient
    ├── socket connection
    ├── packet TX
    ├── RX stream parsing
    └── disconnect handling
```

---

## UI Flow

```text
Navigation Shell
│
├── Console Tab
│   ├── connect panel
│   ├── log output
│   └── ping tools
│
├── Remote Tab
│   ├── joystick controls
│   ├── telemetry HUD
│   └── safety buttons
│
└── Beta Tab
    └── experimental control UI
```

---

## Networking Protocol

Communication is performed over TCP sockets.

### Command packets

```text
C,ARM
C,DISARM
C,KILL
```

### Stick packets

```text
S,seq,thr,yaw,pit,rol
```

Example:

```text
S,42,0.600,-0.120,0.300,0.000
```

Transmission rate:

```text
50 Hz (every 20 ms)
```

---

## Current File Structure

```text
lib/
│
├── main.dart
│
├── controllers/
│   └── drone_controller.dart
│
├── services/
│   └── tcp_client.dart
│
├── screens/
│   ├── drone_tcp_console.dart
│   ├── drone_remote_control.dart
│   └── beta_remote_preview.dart
│
└── widgets/
    ├── connect_panel.dart
    └── log_view.dart
```

---

## Current Development Goals

* shared Provider architecture
* live telemetry from ESP32
* battery status HUD
* altitude + IMU display
* packet checksum validation
* smoother joystick spring animation
* camera / FPV feed integration

---

## Tech Stack

* Flutter
* Dart
* Provider
* TCP sockets
* ESP32 firmware backend

---

## Purpose

This project is being developed as a **real-time drone control GUI prototype** to be used by Washington 
Aerial Robotics. 
