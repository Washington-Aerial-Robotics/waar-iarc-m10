# Full-stack verification matrix

Treat each numbered stage as a gate. Record date, operator/reviewer, Git commit,
firmware hash, Ubuntu image version, ROS domain, hardware serials, command
output, logs, and pass/fail disposition. A later stage cannot waive a failure in
an earlier one. Any unexpected motor output, stale-state command, uncontrolled
motion, frame jump, NaN, process crash, thermal/power warning, or loss of the
independent kill path is an immediate abort.

## Global acceptance rules

- The service never auto-arms. `AUTO_ARM=false` is required at every stage.
- A missing/stale map, transform, state estimate, acknowledgement, camera, or
  command stream results in hold/land/disarm behavior defined by the current
  state; it never reuses an old trajectory indefinitely.
- State and command frames are explicit. `odom` data is not relabeled as `map`;
  the `map -> odom -> base_link` transform must be available before mapped
  navigation.
- The 12-float legacy state estimate's `q.x/q.y/q.z` values are not treated as a
  ROS quaternion. Orientation must be reported unknown unless a valid normalized
  quaternion comes from the new telemetry path.
- Only the `P` sender controls streamed Pi trajectories, and the destination is
  firmware ID `U`. Message headers, little-endian payloads, sequence handling,
  and timeouts must match the firmware built from the recorded commit.
- Firmware battery percentage is currently invalid (the historical value was a
  hardcoded 100), so its low-battery failsafe is not operational. Props-off
  thrust, tether, and free flight are blocked without an independently verified
  battery alarm/cutoff and conservative flight-time limit, or a tested real
  firmware battery measurement.
- Firmware control calibration must be explicitly valid at runtime before Pi
  arming. The invalid-calibration status/default `hoverThrust=0` is a blocker;
  never substitute a guessed default.
- A test passes only when its objective evidence is saved. "It looked okay" is
  not an acceptance criterion.

## Gate 0 — configuration and traceability

| ID | Procedure | Acceptance criteria | Abort / evidence |
|---|---|---|---|
| G0.1 | Run `git status --short`, `git rev-parse HEAD`, and inspect `install/waar-build-manifest.txt`. Record the ESP32 binary SHA-256 and source commit. | Checkout and manifest are clean and name the reviewed commit; firmware image maps to one reviewed source revision. | Abort on dirty/untraceable code. Save command output and hashes. |
| G0.2 | Diff `/etc/waar-drone/$USER.env` against the example without publishing it to a shared log. | `AUTO_ARM=false`; initial `DRY_RUN=true`; `P -> U`; TCP host/port match firmware; camera path is stable; ROS domain is unique at the test site. | Abort on placeholder, unexpected ID/port, duplicate domain, or world-readable env file. |
| G0.3 | Review mass, battery, center of gravity, fasteners, prop direction (props still removed), guards/tether, kill/power path, and test roles. | Two people sign off: flight operator and safety observer; kill path is reachable and independent of ROS/Wi-Fi. | Abort on single-person flight test or inaccessible kill. |
| G0.4 | Survey the arena datum, origin, +X/+Y axes, Z convention, bounds, and launch pose using an independent measurement. Configure the transform tying SLAM's local map to that surveyed arena frame. | A signed survey record is named by `ARENA_FRAME_SURVEY_ID`; known control points transformed through ROS agree with survey within the team's predeclared tolerance; axes are right-handed and no arbitrary RTAB-Map startup origin is treated as arena coordinates. | Abort on missing datum transform, mirrored/rotated axes, reset-induced origin change, or out-of-tolerance checkpoints. Save survey and TF samples. |
| G0.5 | Measure the physical `base_link -> camera_*` translation/rotation on the assembled aircraft, including ROS optical-axis convention. Compare the URDF baseline with the calibration projection (`baseline = -P_right[3]/P_right[0]`) and the independent physical baseline. | `CAMERA_EXTRINSICS_ID` and `STEREO_CALIBRATION_ID` identify signed records; URDF, `right.yaml`, and physical baseline agree within a declared tolerance; calibration resolution/camera/rectification match the flight device. | Abort on copied/unknown extrinsics, baseline sign error, calibration-camera mismatch, or optical-frame convention error. Save measurements and calculation. |
| G0.6 | Measure the black AprilTag square on the exact printed marker (exclude border/quiet zone) at several axes/markers; configure `APRILTAG_SIZE_M`. Validate pose against tags placed at surveyed ranges/angles. | Placeholder is absent; configured mean and uncertainty are recorded; recovered range and map position meet predeclared error limits across the working envelope. | Abort on unmeasured/default size, print scaling variation, or systematic depth/axis bias. |
| G0.7 | Establish true/magnetic/grid north policy, magnetic declination, flight-controller yaw convention, sensor mounting rotation, and map-axis alignment. Rotate the restrained aircraft through known headings away from ferrous objects. | `YAW_ALIGNMENT_ID` identifies a signed record; yaw direction, wrap at ±pi, declination, and `map/base_link` alignment are correct within tolerance; no motor-current test invalidates magnetometer calibration. | Abort on 90/180-degree offset, reversed yaw, discontinuous wrap, magnetic interference, or undocumented yaw source. |
| G0.8 | Install and bench-test an independent battery voltage monitor/alarm or cutoff using measured voltages and a controllable supply/battery simulator. Establish a chemistry-, load-, and capacity-appropriate warning/abort threshold and conservative maximum flight time. | `BATTERY_SAFETY_ID` names the signed record; alarm/cutoff triggers at the recorded threshold independent of Pi/Wi-Fi/ROS; `MAX_FLIGHT_TIME_S` is below the demonstrated safe loaded duration with documented reserve. Firmware's BATTERY_VALID=false remains visible, not masked. | Block all thrust/tether/free-flight tests if relying on firmware's hardcoded/invalid percentage, if the alarm is inaudible/unreachable, or if no reserve is demonstrated. Never deep-discharge a real pack for this test. |
| G0.9 | Perform the firmware's documented aircraft-specific control calibration with props removed or in an approved calibration fixture. Verify stored values and status flag after cold boots; do not enter a value copied from another aircraft. | `CONTROL_CALIBRATION_ID` names the signed record; runtime telemetry reports calibration-valid flag 0x40; hover thrust is finite, positive, aircraft-specific, and within a reviewed plausible range; Pi prepare/arm fails closed when flag is cleared. | Abort on default/zero/NaN/copied calibration or any path that arms while calibration invalid. No document should prescribe a generic hover-thrust number. |

## Gate 1 — static and unit tests (no hardware)

| ID | Procedure | Acceptance criteria | Abort / evidence |
|---|---|---|---|
| G1.1 | `bash deploy/ubuntu/tests/test_deploy.sh` | Syntax passes; protocol/launch defaults match; wrapper rejects auto-arm and unacknowledged hardware mode; installer contains no enable/start action. | Save stdout; any failure blocks deployment. |
| G1.2 | `python3 -m compileall -q Autonomy` and parse every `package.xml`, URDF, and YAML file. | No syntax/XML/YAML errors. | Save exact failing filename/line. |
| G1.3 | Run pure Python suites: `pytest -q Autonomy/waar_perception/test`, bridge protocol/planner tests, `Autonomy/mas_coordinator/tests`, and `Autonomy/waar_autonomy/tests`. | All selected tests pass from a clean environment. Skips require written justification and may not cover flight protocol or failsafes. | Save JUnit/pytest output. |
| G1.4 | `colcon build --base-paths Autonomy --merge-install`; then `colcon test` and `colcon test-result --verbose`. | All flight-stack packages build; zero test failures. | Warnings affecting protocol, frames, QoS, timing, or deprecated APIs block approval. |

Required protocol unit cases:

1. Encode/decode golden bytes for the 4-byte header, state request/reply, flight
   mode, origin command, and trajectory payload (`uint32` sequence plus seven
   little-endian finite `float32` values; exactly 32 payload bytes).
2. Reject wrong destination/source, unknown message, wrong length, truncated and
   coalesced TCP data, non-finite floats, invalid modes, and stale/out-of-order
   sequences. Verify deliberate sequence recovery after a Pi restart only in the
   firmware-approved disarmed state.
3. Exercise receive fragmentation at every byte boundary and reconnect between
   header and payload. No partial packet may mutate a command.
4. Verify wraparound behavior near `UINT32_MAX`, duplicate handling, response ID
   correlation, failure replies, bounded retry, and no busy loop on EOF.
5. Verify the bridge reports unknown orientation for legacy tilt/yaw telemetry;
   if a quaternion endpoint is used, reject non-finite/zero norm and normalize a
   valid quaternion.
6. Verify every mission command (`HOLD`, `STANDBY`, `AWAIT`, `SWEEP_SECTOR`,
   `FILL_GAPS`, `VERIFY_TAG`, `VERIFY_PATH`, `LAND`, and unknown JSON) has an
   explicit safe outcome. Unknown/malformed commands must hold.
7. Verify occupancy-grid A* rejects stale/missing maps and blocked goals, obeys
   inflation and unknown-cell policy, remains inside arena bounds, and produces
   deterministic collision-free paths for fixed fixtures.
8. Verify AprilTag projection through known camera/TF fixtures, arena rejection,
   stable tag IDs, monotonic sequence, confidence thresholds, stale-TF failure,
   and no automatic "confirmed" result without close high-confidence evidence.

## Gate 2 — fake-ESP integration (loopback, no hardware)

Run a deterministic fake TCP server on loopback and launch only the bridge or
the unified stack with its hardware endpoint redirected to that server. The
fake server must capture every received byte and allow scripted delay, malformed
reply, fragmentation, and disconnect behavior.

| ID | Stimulus | Acceptance criteria |
|---|---|---|
| G2.1 | Start with `DRY_RUN=true`, no mission, and valid periodic telemetry. | Bridge publishes typed odometry/IMU/pose; it sends no arm or trajectory packet; lifecycle state is not flight-ready until explicit prepare. |
| G2.2 | Fragment each reply, join multiple replies in one TCP write, inject unknown packets, and delay within the allowed response window. | Parser recovers without desynchronizing, corrupting state, or blocking the ROS executor. Unknown packets are logged/rate-limited and ignored. |
| G2.3 | Supply NaN/Inf, impossible lengths, wrong IDs, stale sequence/response IDs, EOF mid-packet, and connection refusal. | Bad data is never published/commanded; bridge enters hold/not-ready, retries with bounded backoff, and exposes an actionable error. CPU remains bounded. |
| G2.4 | Restart fake ESP, then restart the bridge. Test sequence reset under disarmed conditions and rollover. | Reconnection is deterministic; no old setpoint is replayed; first accepted command follows the documented firmware recovery rule. |
| G2.5 | With hardware mode directed only to the fake server, explicitly call prepare then arm, publish a bounded planner setpoint, and inspect captured bytes. | Exact expected packets and cadence; no setpoint before prepare/arm; coordinates/yaw/velocities are finite and frame-correct. |
| G2.6 | Stop planner setpoints, withhold replies, drop TCP, kill the bridge, and stop the launch in separate runs. | Command output stops within the configured deadline; expected hold/land/disarm request occurs where possible; no stale trajectory continues. |

Save the fake-server transcript and a ROS bag. Repeat G2.5/G2.6 at least 20
times to expose reconnect/race failures.

## Gate 3 — ROS full-stack simulation/dry run

Start the unified launch with a recorded or simulated stereo source and fake ESP.

| ID | Procedure | Acceptance criteria |
|---|---|---|
| G3.1 | Run `verify_stack.sh --allow-manual --wait 90`. Inspect `ros2 node list`, `topic list -t`, `service list -t`, and `tf2_echo map base_link`. | One instance of each intended node; exact message/service types; continuously fresh state/camera/map/TF; no legacy CSV bridge or four-drone stub. |
| G3.2 | Replay stationary sensor data for 10 minutes. | No pose explosion, NaN, unbounded covariance, TF loop, memory growth, or command output. Drift stays within the team-defined stationary bound. |
| G3.3 | Replay a mapped route with loop closure. | Map is coherent; `map->odom` changes smoothly enough for controller limits; planner setpoints remain continuous and in bounds. |
| G3.4 | Inject all mission command fixtures, including malformed/unknown JSON and pre-map commands. | Deterministic paths for valid commands; hold for invalid, unknown, no-map, no-TF, and no-path cases; `LAND` preempts navigation. |
| G3.5 | Replay known AprilTags inside/outside the arena with correct, stale, and absent TF. | Only valid mapped candidates publish; IDs/sequence/confidence are stable; verification results match explicit close evidence; outliers and stale TF fail closed. |
| G3.6 | Stress at target rates for 30 minutes while recording a bag. | No missed watchdog cadence, sustained CPU saturation, thermal throttling, growing queue/memory, or timestamp regression. Set quantitative CPU/RAM/rate limits before running. |

## Gate 4 — Ubuntu hardware preflight (powered, props removed)

Stop the ROS bridge, remove props, restrain the frame, confirm disarmed on the
flight controller, and run:

```bash
AIRFRAME_DISARMED_ACK=PROPS_REMOVED_AND_DISARMED \
  deploy/ubuntu/preflight.sh --env-file /etc/waar-drone/"$USER".env \
  --hardware --probe-esp32
```

| ID | Procedure | Acceptance criteria |
|---|---|---|
| G4.1 | Cold boot three times; list camera/optional serial links and run preflight. | Stable links resolve to the same physical devices; correct permissions; camera advertises calibrated mode; no USB resets in `dmesg`. |
| G4.2 | Check `ip route get`, Wi-Fi association, ping, then the guarded TCP probe with service stopped. | Correct interface/subnet, no address collision, acceptable RSSI/site noise, TCP accepts exactly one client. A second client cannot displace an active bridge silently. |
| G4.3 | Check NTP, free space, CPU temperature/throttling, battery/power rail under camera+compute load. | Clock synchronized; >3 GiB free; no undervoltage/throttling/USB brownout; temperatures remain below platform limit with margin. |
| G4.4 | Start in `DRY_RUN=true` and run `verify_stack.sh`. | Full telemetry/camera/map graph passes; captured ESP traffic contains no control/arm packet. |

## Gate 5 — props-off control and failsafe HIL

Set `DRY_RUN=false` only with the acknowledgement described in the README. Props
remain removed and the frame restrained. Record ROS bag, firmware serial, packet
capture/fake-ground-station view, and video. Have one operator at the kill switch.

| ID | Procedure | Acceptance criteria / abort |
|---|---|---|
| G5.1 | Start service and wait 60 s without calling prepare/arm. | Controller remains disarmed and motors produce zero output. Any spin is immediate power-off/abort. |
| G5.2 | Call `/d1/prepare`; separately verify sensor freshness, origin latch, mode, and firmware reply. | Prepare succeeds only with all required fresh/valid state and does not arm. Failure response is truthful and leaves vehicle disarmed. |
| G5.2a | Observe the new telemetry validity/status bits before each prepare/arm attempt; test invalid control calibration and battery-invalid reporting deliberately while props are removed. | Bit 0x40 accurately gates control-calibration readiness; cleared 0x40 blocks prepare/arm. Bit 0x80 remains clear until real firmware battery sensing exists; the bridge/operator reports that limitation and never interprets legacy 100 as valid charge. Independent alarm/timer is active before any thrust. |
| G5.3 | With observer approval, call `/d1/arm`, then command zero-velocity hold and small bounded setpoints one axis at a time. | Explicit arm is required; motor response/sign matches expected axes; setpoints are finite, rate-limited, and acknowledged. Stop immediately on wrong motor/order/sign. |
| G5.4 | Stop setpoints for longer than the firmware streaming deadline. | Firmware exits streamed control and enters the documented safe action within deadline; no old trajectory remains active. |
| G5.5 | Pull Wi-Fi / block TCP, kill the ROS bridge, kill the planner, and freeze telemetry in separate runs. | Every single fault causes bounded hold/land/disarm behavior; no process restart auto-arms or resumes an old command. Measure detection/action latency. |
| G5.6 | Send duplicate/stale/out-of-order sequences and invalid setpoints using the approved test harness. | Firmware returns failure and does not mutate the active command. Fresh valid traffic can recover only through the documented state transition. |
| G5.7 | Call land, then disarm; interrupt service during each lifecycle phase in separate runs. | Land preempts navigation; disarm is accepted only when safe; shutdown never leaves autonomous thrust. |
| G5.8 | Reboot Pi and ESP independently while disarmed, then together. | Service reconnects without automatic arm; sequence/origin recovery works; readiness must be re-established explicitly. |

Repeat every fault case at least three times. Any timing miss, unexplained reply,
motor output after timeout, or recovery that bypasses prepare/arm blocks props.

## Gate 6 — restrained/tethered thrust test

Install inspected props only in a net/cage or rated tether fixture with a clear
exclusion zone. Use a reduced thrust/altitude limit reviewed for the fixture.
Never rely on a hand-held airframe as restraint.

| ID | Procedure | Acceptance criteria |
|---|---|---|
| G6.1 | Arm and hold the lowest safe thrust briefly, then land. | Correct motor order/direction, no excessive vibration/current, estimator remains finite, kill and land work. |
| G6.2 | Small roll/pitch/yaw/vertical steps within fixture limits. | Correct sign, bounded overshoot, no oscillation/saturation, setpoint and measured response align in logs. |
| G6.3 | Planner heartbeat loss and Wi-Fi loss, one at a time, at the lowest safe energy. | Failsafe latency/action match G5 and the fixture remains within limits. Kill on any deviation. |

Require independent review of plots for command/state timing, position/velocity,
attitude validity, motor saturation, packet loss, and failsafe latency.

## Gate 7 — first free hover and incremental autonomy

Use a controlled indoor flight area, guards as appropriate, two operators,
pre-briefed abort words, a hardware kill path, and a conservative geofence.

| ID | Procedure | Acceptance criteria |
|---|---|---|
| G7.1 | Manual explicit prepare/arm, take off to minimum safe altitude, hold 10 s, land. No mission. | Stable hover inside bound; valid/fresh state and TF; no saturation, major drift, or dropped control cadence. |
| G7.2 | Repeat hold for 30 s and perform very small bounded cardinal moves, returning to center after each. | Correct direction, bounded error/overshoot, geofence respected, operator land works immediately. |
| G7.3 | Run one short pre-reviewed planner path with perception disabled. | Collision-free bounded trajectory, no discontinuities, clean land. |
| G7.4 | Enable perception and map with inert/HOLD mission, then a single short sweep lane. | Map/TF stable under flight vibration; detections remain in arena; no unexpected task or path transition. |
| G7.5 | Run a shortened end-to-end mission including one explicit verification and land. | Every state transition matches logs; result publishes only on explicit evidence; mission timeout/land preempts all tasks. |

Increase duration, speed, altitude, and mission complexity one variable at a
time. After each flight inspect logs before continuing. A software pass never
overrides the safety observer's abort.

## Abort, incident, and rollback procedure

1. Call land if control remains trustworthy; use the independent kill/power path
   if it does not. Do not troubleshoot an energized uncontrolled vehicle.
2. After touchdown, verify zero thrust and disarmed state independently, remove
   battery power, then stop the service.
3. Preserve ROS bag, firmware serial, systemd journal, diagnostics directory,
   operator video, exact configuration, hashes, and a UTC incident timeline.
4. Do not rerun until the cause is understood, a regression test reproduces it,
   the fix passes all earlier gates, and a reviewer signs off.
5. For rollback, use the previous paired, known-good Ubuntu checkout and firmware
   image. Restore its root-owned environment file, rebuild, verify hashes, and
   restart at Gate 0. Never mix an old bridge with a new unverified protocol.
