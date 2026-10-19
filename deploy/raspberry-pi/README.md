# One physical drone with an onboard Raspberry Pi 4/5

This deployment runs a **Vicon position companion**, with software checks,
manual bench tools, and a procedure for saving a reusable SD-card image.
The ESP32 remains the flight controller. Installing or starting the companion
does not arm motors, choose a flight mode, or issue flight setpoints.

## What runs where

```text
Vicon cameras -> Tracker PC -> PC Python relay -> onboard Pi -> ESP32 -> ESCs
                 DataStream     UDP 5100          TCP 70     DShot300
                 TCP 801
```

Use one dedicated lab router: PC `192.168.0.10`, Pi `192.168.0.20`, ESP32
`192.168.0.240` are examples, not automatic assignments. Reserve the PC/Pi
addresses on the router; keep the ESP32's fixed address outside its DHCP pool.
Connect the ESP32 to 2.4 GHz. Both computers need a common time source.
Allow PC-to-Pi UDP 5100, Pi-to-ESP32 TCP 70, and the SDK client's connection
to Tracker TCP 801. Leave the Vicon camera-network adapter configuration alone.

The PC needs Vicon's Python SDK compatible with the lab's Tracker version.
The Pi relay uses only Python's standard library. Vicon lists Linux x64
packages; Linux ARM Python support must not be assumed. The PC relay avoids
that dependency. The repo's older setup was tested with Tracker 3.10; Vicon's
1.12 release lists Tracker 3.x, whereas 1.13 lists Tracker 4.x. Use the lab's
working combination. [Vicon SDK requirements/downloads](https://www.vicon.com/software/datastream-sdk/).

Use Raspberry Pi OS Lite **64-bit** on the Pi 4/5. Power it from the model's
specified supply or a suitable onboard regulator, not an ESP32 GPIO/3.3 V pin.
Secure the Pi/cables and check the resulting weight and centre of gravity.
This path uses WiFi between Pi and ESP32; no UART wiring is needed.

## 1. Check the software on Windows

From the repository root:

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-test.txt
.\.venv\Scripts\python.exe scripts/check_software.py
```

This runs companion protocol tests, planner tests and coordinator tests in
separate processes. Network traffic goes only to loopback mock hardware.
It does not execute motor/autotune scripts. ROS2 and Flutter checks are separate.

To run the simulated mission:

```powershell
cd Autonomy/waar_autonomy
..\..\.venv\Scripts\python.exe sim.py --no-anim
cd ../..
```

The simulation moves between grid blocks and reads a simulated hazard map.
A passing run does not validate a real trajectory, sensors, or flight control.

## 2. Build and flash the ESP32, with propellers removed

1. Check the board/motor pin mapping in `ESP32/KAF_Drone/src/auxilary/common_data.h`
   against your wiring.
2. Copy `ESP32/KAF_Drone/src/firmware/wifi_credentials.h.example` to
   `wifi_credentials.h` beside it; fill in the lab router credentials and IPs.
   Keep SSID/password within the example's documented limits. This file is ignored by Git.
3. Install the **Espressif** `esp32:esp32` core and **DShotRMT 0.9.5**.
   DShotRMT 0.6.6 lacks `sendThrottlePercent` and fails to compile.
   Use `soft/firmware_vicon`, not the stale `flashing_examples/firmware_full`
   path in the older flight-controller README.

With Arduino CLI on your PATH (or use these selections in Arduino IDE):

```powershell
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.11 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli lib install DShotRMT@0.9.5
arduino-cli compile --fqbn esp32:esp32:esp32 --library ./ESP32/KAF_Drone ./ESP32/KAF_Drone/soft/firmware_vicon
arduino-cli board list
```

`esp32:esp32:esp32` is the generic classic ESP32 target used for the compile
check. Confirm your board variant, flash size and COM port before uploading;
use that board's FQBN if different. Then upload explicitly, for example:

```powershell
arduino-cli upload --port COM7 --fqbn esp32:esp32:esp32 ./ESP32/KAF_Drone/soft/firmware_vicon
arduino-cli monitor --port COM7 --config baudrate=115200
```

The firmware uses raw DShot zero for motor stop. DShotRMT's zero-percent helper
maps to minimum running throttle, so disabled/arming/zero-output paths use
`sendThrottle(0)` explicitly. Verify stopped motors before fitting props.
[DShotRMT API](https://github.com/derdoktor667/DShotRMT/tree/0.9.5).

Confirm the boot IP and open `http://192.168.0.240`. If router association fails,
the ESP32 falls back after about 20 s to its own AP at `192.168.1.240`; fix the
router settings before continuing. Check saved PID/calibration data for this
airframe. The firmware includes a magnetometer correction for a previous
sensor; it is not a universal calibration.

## 3. Install the Pi companion

1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/documentation/computers/getting-started.html)
   to write Pi OS Lite 64-bit. Select your model, set hostname `waar-d1`,
   configure your user, SSH key, WiFi and country, then boot the Pi.
2. Copy this checkout to the Pi (Git, SCP or USB). Keep its directory structure.
   The installer copies `Autonomy`, `ESP32`, `deploy` and `scripts`, excluding
   the Windows environment and ESP32 WiFi credentials. CAD is not installed.
3. On the Pi, from the checkout root:

```bash
sudo bash deploy/raspberry-pi/install.sh
sudo nano /etc/waar-single-drone.env
```

Installation creates `/opt/waar/venv`, runs the software checks and installs
`waar-companion.service`. It stops an existing instance during an update and
leaves it stopped. Existing configuration is preserved. Set `VICON_RELAY_IP`,
`DRONE_IP`, `DRONE_ID` (normally `U`), and leave `WAAR_MODE=monitor` initially.
The service runs as unprivileged user `waar`.

Synchronize PC and Pi to the same reachable time source before starting:

```bash
chronyc tracking
chronyc sources -v
sudo chronyc waitsync 30 0.01
```

On Windows inspect `w32tm /query /status`. An isolated router needs a reachable
lab NTP server configured on both computers; an installed time service alone
does not guarantee synchronization. Target offsets below 10 ms. Datagrams
with over 80 ms apparent age or more than 20 ms future skew are rejected;
the Pi deducts a 20 ms clock margin from its 100 ms freshness budget. Inspect
the journal's rejected count if no poses arrive.

On the PC, in an environment with the lab's Vicon SDK installed:

```powershell
python -c "from vicon_dssdk import ViconDataStream"
python ESP32/KAF_Drone/tests/vicon/vicon_link.py relay --vicon 192.168.0.10 --object waar1 --pi-ip 192.168.0.20
```

The relay can run on the Tracker PC or another SDK-capable computer. Set
`--vicon` to Tracker and `VICON_RELAY_IP` to the computer sending the UDP packets.
The relay sends the original Vicon world frame; set the yaw offset on the Pi
once. UDP filters the sender's IPv4 address and validates format, numbers,
order and age. It is unauthenticated and intended for the private lab LAN.

On the Pi:

```bash
sudo systemctl start waar-companion
sudo journalctl -u waar-companion -f
```

Logs are in `/var/lib/waar/pose-*.csv`. Each restart creates a new CSV;
archive/remove old runs to keep free space. Use `systemctl status waar-companion`
for state and `sudo systemctl stop waar-companion` to stop it.

## 4. Physical test order and pass criteria

Remove propellers for A-D. Support the airframe and keep the battery disconnect
accessible. The browser Kill button uses HTTP while the Pi owns TCP 70; it
still depends on a working network. Verify physical kill/power-disconnect
arrangements before a powered restrained test.

**A. Motors and IMU, Pi service stopped.** The ESP32 accepts one TCP client.
Close Flutter and stop the Pi service before running these from the PC:

```powershell
python ESP32/KAF_Drone/tests/motor_direction_test.py --ip 192.168.0.240
python ESP32/KAF_Drone/tests/relay_autotune.py --ip 192.168.0.240 check
python ESP32/KAF_Drone/tests/pid_calibrate.py --ip 192.168.0.240 get all
```

The motor test **arms and pulses real motors after its confirmation prompt**.
Pass: M0 front-left CCW, M1 rear-left CW, M2 front-right CW, M3 rear-right CCW
(viewed from above); zero/disarm stops every motor. The IMU check changes
calibration/estimator modes; follow its hand-motion prompts. Pass: reported
body x/front, y/left, z/up match physical motion and motor mixing. Stop if axes,
pairing or direction are wrong; PID tuning cannot fix them.

`relay_autotune.py tune --axis roll` (then pitch/yaw) is a separate **powered,
propeller-on gimbal test**, after these checks, with other rig axes locked and
suitable current gains/hover throttle. It is not an unattended software test.
Use `pid_calibrate.py --ip 192.168.0.240 save` only after validating gains on your rig.

**B. Vicon axes and yaw.** Calibrate cameras, set the floor origin Z-up, create
asymmetric marker object `waar1` facing world +X, and boot the drone facing +X.
Start the PC relay and Pi service in `monitor` mode. Hand-carry/rotate the
airframe and check x/y/z signs and yaw agreement. Fix drift/jumps before
flight: Vicon yaw is stored but not fused into ESP32 attitude. See the
[detailed Vicon instructions](../../ESP32/KAF_Drone/tests/vicon/README.md).

**C. Position bridge.** Set `WAAR_MODE=bridge` in `/etc/waar-single-drone.env`:

```bash
sudo systemctl restart waar-companion
sudo journalctl -u waar-companion -f
```

Pass: `sent` and `ok` keep increasing, failures stay zero, no stale warnings,
and ESP32 position follows the hand-carried drone within your lab tolerance
(a few cm is the existing guide's target). Compare CSV traces. ACKs alone do
not prove correct axes, tuning or position. Check sustained frame gaps
comfortably below 100 ms throughout the intended tracking volume.

**D. Dropout/restart, still propellers off.** Stop the PC relay, occlude markers,
and disconnect/reconnect the Pi network in separate tests. Pass: Pi forwarding
stops when pose freshness expires, ESP32 invalidates mocap 150 ms after its
last accepted packet, and position-mode entry is refused. Wait over one second
before source restart for firmware sequence recovery. Verify invalid-position/
mode-entry behavior on hardware; the bridge's state reply does not report it.

The firmware attempts landing on invalid position only in the applicable
armed commander/position modes. Manual/acceleration modes do not gain this
protection. Altitude after Vicon loss is dead-reckoned, so lost-link landing
is not a verified independent landing system.

**E. Restrained powered test, then low hover.** Use the appropriate gimbal,
tether/net, a clear volume and an operator able to kill power. Confirm restoring
torque, saved gains, yaw, power/temperature and networking with the Pi installed.
Advance to a small position-hold test only after restrained checks pass.
No supplied Python test certifies flight.

**Do not use built-in square/qualification as the first indoor test.** Defaults
command 6 m altitude, a 5 m square side or 5 m orbit radius. Circle code also
documents a trajectory coefficient conflict. These routines need lab-sized
trajectories and dedicated code/hardware review. Square's interaction with the
generic 500 ms Pi-setpoint watchdog remains unresolved in this checkout.

`bridge --fake` / `relay --fake` generate a fictitious circle at 1 m altitude.
Use only without props or with mock hardware; never as flight position input.

## 5. Save an image you can flash again

Provision and bench-test a real Pi, then capture its powered-off SD card.
This creates a bootable `.img` containing the OS, companion, configuration and
services. The installer alone is not a disk image.

On the Pi, record tested versions and set the desired boot mode:

```bash
sudo /opt/waar/venv/bin/python /opt/waar/repo/scripts/check_software.py
dpkg-query -W python3 python3-numpy python3-pytest chrony
sha256sum /opt/waar/repo/ESP32/KAF_Drone/tests/vicon/*.py
# Keep WAAR_MODE=monitor for the first reusable image.
sudo systemctl enable waar-companion
sudo poweroff
```

After shutdown remove the card. On another Linux machine (or Linux live USB),
insert it, identify it using `lsblk -o NAME,SIZE,MODEL,TYPE,MOUNTPOINTS`, unmount
each mounted partition, and run:

```bash
# EXAMPLE: replace /dev/sdX with the positively identified source SD disk.
sudo bash deploy/raspberry-pi/capture-image.sh /dev/sdX /path/with/enough/space/waar-single.img
```

The helper reads the whole card into a new file and writes a SHA-256 checksum.
It refuses mounted/partition sources, existing outputs and device outputs.
Budget disk space for the full card capacity. It does not shrink the image;
replacement cards need at least the same byte capacity. The image contains
SSH keys, credentials, WiFi settings, hostname and logs: keep it private for
this drone. Archive ESP32 firmware/core/library versions separately; a Pi
image does not flash or back up ESP32 firmware or EEPROM calibration.

In Raspberry Pi Imager on Windows choose **Use Custom**, select this `.img`,
write a spare card and let Imager verify it. Boot the spare on the intended Pi
model, props removed, and repeat B-D. Pi 4 and Pi 5 each need their own actual
boot/hardware checks. A service enabled in bridge mode resumes position
forwarding at boot when its PC relay is available; it still never arms.

## What is missing for real autonomous exploration

This image is a position companion and bench-test environment. Flying the
corridor planner still requires:

- A real sensor/detection adapter replacing the simulated hazard map.
- A bounded, continuous trajectory executor replacing grid-block teleportation,
  with Vicon-to-map origin/scale, altitude and geofence limits.
- One TCP 70 owner combining mocap, telemetry and `COM_SET_TRAJSETPT`, handling
  ACKs, the 500 ms setpoint watchdog, disarm/land and source restarts.
- Hardware-verified control/dropout behavior. The ESP32 TCP reader can discard
  coalesced packets; send spacing mitigates this but is not reliable stream framing.

The ROS2 README references `ros2_adapter_v2.py` / `BaselineLoop`, absent here.
`team_launch.py` starts d1-d4 even with `num_drones:=1`. It is not a single-drone
hardware launch. ROS2 is not needed for this position companion.

## Validation record

On 2026-10-18, Windows / Python 3.14: 7 companion, 43 planner and 94 coordinator
tests passed; 4 adapter tests were skipped because their expected adapter
checkout is unavailable. The loopback test
checks source-dropout suppression, coordinate rotation and absence of arming
commands. It does not emulate flight dynamics or ESP32 TCP timing defects.

`firmware_vicon` compiled for `esp32:esp32:esp32` with core 3.3.11 and DShotRMT
0.9.5: 1,090,836 bytes flash (83%) and 62,836 bytes global RAM (19%). The
headless planner reached certification at tick 263 (seed 42).

Pi installation, SD capture/restore and flight require physical devices and
have not been executed here. Record OS/SD version, Pi model, board, airframe,
firmware revision, calibration and test logs alongside your captured image.
