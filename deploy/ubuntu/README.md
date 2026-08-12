# One-drone Ubuntu deployment

This directory installs and runs one copy of the autonomy stack on Ubuntu 22.04
with ROS 2 Humble. The default configuration is deliberately inert:
`DRY_RUN=true`, `AUTO_ARM=false`, and the systemd unit is neither enabled nor
started by the installer. No script here flashes the ESP32 or arms a vehicle.

Passing software tests is necessary, but it is not flight approval. Complete
the gates in [TEST_MATRIX.md](TEST_MATRIX.md) in order. In particular, do not
install propellers until every props-off hardware-in-the-loop test passes.

## 1. Freeze the release to be tested

Use a dedicated checkout on the drone. Substitute the actual repository URL;
do not copy a URL or credentials into the service environment file.

```bash
git clone <REPOSITORY_URL> waar-iarc-m10
cd waar-iarc-m10
git switch ground-station/communication-pipeline
git pull --ff-only
git status --short
git rev-parse HEAD
```

`git status --short` must be empty for a flight candidate. Record the full
commit hash together with the ESP32 firmware commit/hash. Never run an
unattended `git pull` from systemd; a release should not change between boot and
flight.

The computer must already have Ubuntu 22.04 and ROS 2 Humble installed. Confirm
that `/opt/ros/humble/setup.bash` exists. The build script intentionally refuses
a different OS/ROS pairing rather than silently installing a second platform.

## 2. Install dependencies and build

Run as the Linux account which will own the service, not as root:

```bash
bash deploy/ubuntu/install_on_drone.sh --run-tests --install-systemd
```

This runs `rosdep`, discovers packages under `Autonomy`, performs a release
`colcon build`, runs package tests, records `install/waar-build-manifest.txt`,
and installs a service template. It uses `sudo` only for apt, rosdep
initialization, and the optional service files. It does not flash, enable,
start, prepare, arm, or command the drone.

For an offline rebuild after dependencies are installed:

```bash
bash deploy/ubuntu/install_on_drone.sh --skip-apt --skip-rosdep --run-tests
```

## 3. Identify devices without guessing

The stereo camera should use a stable `/dev/v4l/by-id/...` link:

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
v4l2-ctl --device /dev/v4l/by-id/<CAMERA_LINK> --list-formats-ext
```

Verify that the exact device advertises the calibrated combined stereo mode
(`2560x960` at the configured rate by default). A `/dev/video0` name can change
after a reboot or USB reconnect and is not acceptable for flight.

The current flight bridge communicates with the ESP32 over TCP at the address
and port compiled into the firmware (`192.168.1.240:70` in this branch). The
sender and receiver IDs are the one-byte ASCII IDs `P` and `U`. The firmware
accepts only one TCP client: do not run a diagnostic client while the ROS bridge
is connected.

The old CSV serial bridge is disabled. Leave `BRIDGE_SERIAL_PORT` and
`SENSOR_SERIAL_PORT` empty for the TCP deployment unless a separate device is
intentionally configured and documented. If stable device
links do not exist, use `udevadm info --attribute-walk` and adapt
[99-waar-drone.rules.example](99-waar-drone.rules.example) with the exact vendor,
product, and serial attributes. Never install the example with placeholders.

## 4. Configure the inert service

The installer places an example, not an active environment file:

```bash
sudo cp /etc/waar-drone/"$USER".env.example /etc/waar-drone/"$USER".env
sudoedit /etc/waar-drone/"$USER".env
sudo chown root:"$(id -gn)" /etc/waar-drone/"$USER".env
sudo chmod 0640 /etc/waar-drone/"$USER".env
```

Set `REPO_DIR` to the absolute checkout and replace the camera placeholder.
Measure the marker's black square (not the white quiet zone or paper) and set
`APRILTAG_SIZE_M`; leaving the placeholder blocks launch. Set each `*_ID`
calibration/survey field to the approved record for this aircraft and venue.
Set `ARENA_MAP_ALIGNED=true` only after that signed survey validates the actual
`map` transform; leaving it false safely blocks launch.
The current firmware does **not** provide a valid measured battery percentage;
its telemetry marks battery invalid. Before any motor/flight test, provide an
independent, verified low-voltage monitor/alarm or cutoff, name its validation
record in `BATTERY_SAFETY_ID`, and configure a conservative tested
`MAX_FLIGHT_TIME_S`. A ROS timer is not a substitute for the independent device.
Also complete aircraft-specific runtime control calibration and name its record
in `CONTROL_CALIBRATION_ID`; the firmware's invalid/default calibration blocks
Pi arming, and this deployment intentionally provides no guessed hover-thrust
value.

Keep `REQUIRE_BATTERY_VALID=true` when real firmware battery sensing is
available. With the current invalid battery telemetry, it may be set false only
after the independent monitor/cutoff record and conservative flight-time limit
are configured and reviewed; this bypass does not make the firmware battery
failsafe operational.
Keep these values for the first launch:

```text
DRY_RUN=true
AUTO_ARM=false
HARDWARE_ENABLE_ACK=disabled
TRANSPORT=tcp
SENDER_ID=P
ESP32_DEVICE_ID=U
```

Check the generated launch command without starting ROS:

```bash
/usr/local/libexec/waar-drone/run_one_drone.sh \
  --env-file /etc/waar-drone/"$USER".env \
  --print-command
```

Run the installation preflight:

```bash
/usr/local/libexec/waar-drone/preflight.sh \
  --env-file /etc/waar-drone/"$USER".env
```

Every failure must be resolved. Warnings require a written disposition in the
test log.

## 5. Dry-run launch

Start the service manually. Do not enable it at boot yet.

```bash
sudo systemctl start "waar-drone@$USER.service"
systemctl status "waar-drone@$USER.service" --no-pager
journalctl -fu "waar-drone@$USER.service"
```

In another terminal, perform the read-only graph check:

```bash
/usr/local/libexec/waar-drone/verify_stack.sh \
  --env-file /etc/waar-drone/"$USER".env \
  --wait 90
```

The checker never publishes a command or calls a lifecycle service. A passing
result means that expected data and interfaces exist; it does not prove control
authority or safe flight behavior.

Stop the dry run:

```bash
sudo systemctl stop "waar-drone@$USER.service"
```

## 6. Props-off ESP32 check

Physically remove all propellers, restrain the frame, keep a hardware kill/power
disconnect within reach, and independently confirm that the flight controller
reports disarmed. Stop the ROS service before probing because the firmware has
one TCP client slot.

```bash
AIRFRAME_DISARMED_ACK=PROPS_REMOVED_AND_DISARMED \
  /usr/local/libexec/waar-drone/preflight.sh \
  --env-file /etc/waar-drone/"$USER".env \
  --hardware --probe-esp32
```

The TCP probe connects and immediately closes without sending a protocol
packet. Do not run it on an armed vehicle. Then follow the props-off cases in
[TEST_MATRIX.md](TEST_MATRIX.md), including setpoint timeout, link-loss,
explicit land, and explicit disarm. Do not skip directly to hover.

Only after that gate passes may the operator set:

```text
DRY_RUN=false
HARDWARE_ENABLE_ACK=I_ACCEPT_HARDWARE_OUTPUT
AUTO_ARM=false
```

The wrapper always rejects `AUTO_ARM=true`. Preparation and arming remain
deliberate service calls by an operator under the staged test procedure. A
normal production startup therefore connects and publishes state, but it does
not arm the vehicle.

## 7. Logs and shutdown

Start a ROS bag before any HIL run. Choose a filesystem with enough space:

```bash
source /opt/ros/humble/setup.bash
source <REPO_DIR>/install/setup.bash
export ROS_DOMAIN_ID=27
ros2 bag record -o waar-hil-$(date -u +%Y%m%dT%H%M%SZ) \
  /esp32/odometry /imu/data /d1/pose /tf /tf_static /map \
  /d1/mission_cmd /d1/planner_cmd /d1/mine_candidates \
  /d1/verification_result /team/task_result
```

Also retain the ESP32 serial log and a video of the airframe and operator. After
a failure or abort, collect host/ROS diagnostics:

```bash
/usr/local/libexec/waar-drone/collect_diagnostics.sh \
  --env-file /etc/waar-drone/"$USER".env \
  --output-parent "$PWD"
```

For an intentional shutdown while hardware output is enabled: command land,
visually verify touchdown and zero motor output, command disarm, independently
verify the controller is disarmed, and only then stop the service. Do not use
`systemctl stop` as the primary landing method. A process/link failure must be
covered by the independently tested firmware failsafe.

## Service enablement and rollback

Enable boot startup only after the complete bench, tether, and first-hover gates
pass and the environment file still has `AUTO_ARM=false`:

```bash
sudo systemctl enable "waar-drone@$USER.service"
```

Keep the previous known-good checkout and firmware binary by immutable commit
hash. To roll back, land and power down first, stop/disable the service, point
`REPO_DIR` at the previous tested checkout (or restore the previous root-owned
environment file), rebuild that checkout, restore the matched firmware image,
and rerun preflight from the beginning. Never switch software or firmware while
the vehicle is powered for flight.
