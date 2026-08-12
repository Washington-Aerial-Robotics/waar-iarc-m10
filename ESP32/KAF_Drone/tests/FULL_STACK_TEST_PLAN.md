# ESP32 autonomy integration test plan

This plan separates checks that can run on a development host from tests that need the real ESP32,
sensors, radios, ESCs, and airframe. A passing source-contract test is not hardware verification.

## Protocol contract

- ESP32 device ID: `U` (`0x55`); Ubuntu/Pi controller ID: `P` (`0x50`); TCP port: `70`.
- Packet header: `<to_id, from_id, message_type, message_id>`, one byte each.
- Success/failure replies: `COM_SUCCESS=0x3c`, `COM_FAILURE=0x3d`; reply message ID echoes the request.
- GPS-origin request: type `0x64`, no payload. It succeeds only from `P`, while disarmed, with a fresh GPS
  fix meeting the firmware satellite/HDOP gates. Success zeros position/velocity and resets the Pi stream
  sequence/session. A restarted Pi may then begin again at sequence 1.
- Pi setpoint: type `0x5d`, payload `struct.pack("<I7f", sequence, x, y, z, yaw, vx, vy, vz)` (32 bytes).
  It succeeds only from `P`, with finite fields, fresh position, and a strictly increasing sequence.
- Safe land request: `COM_SET_TRAJECTORY=0x5b`, one-byte payload `FLIGHTPATH_LAND=0x02`. Sender `P` is
  authorized only for LAND; other legacy paths remain ground-station-only. LAND preserves current actuation:
  an airborne craft descends, while a disarmed craft stays disarmed (trajectory selection never auto-arms).
- Stream mode: `COM_SET_FLIGHTMODE=0x4c`, payload bytes `0x0e,0x00` for
  `CMD_NOMINAL_MODE|POS_SETPOINT_MODE`, reusing the already accepted Pi target.
- Arm: `COM_SET_ACTUATION=0x4a`, payload `0xff`; disarm payload `0x00`.
  Only `P` and `G` may change actuation. A Pi arm also requires a measured control calibration; the default
  `hoverThrust=0` deliberately fails that gate. `COM_SET_KILL` remains the sender-independent emergency path.
- Telemetry request: `0x65`, no payload. Reply type is `0x25`, payload format
  `<BBBBI3f3f4f3ff` (64 bytes): protocol version, flight mode, flags, reserved, accepted sequence,
  xyz position, xyz velocity, ROS xyzw quaternion, xyz angular velocity, battery.

Telemetry flag bits: position valid `0x01`, origin set `0x02`, actuating `0x04`, Pi stream selected
`0x08`, Pi setpoint fresh `0x10`, quaternion valid `0x20`, control calibrated `0x40`, battery valid `0x80`.

There is currently no battery measurement source, so `0x80` stays clear and the placeholder battery float
must be ignored; low-battery landing is not operational. Sender IDs are allowlists, not cryptographic
authentication, and can be spoofed by a client already on the AP.

## Stage 0 — host checks (no hardware claim)

Run:

```bash
python3 -m unittest discover -s ESP32/KAF_Drone/tests -p 'test_*.py' -v
arduino-cli compile --fqbn esp32:esp32:esp32 ESP32/KAF_Drone/soft/firmware_full
```

Pass: all contract tests pass and the exact sketch/library revision compiles with the same ESP32 Arduino
core used for the flight controller. Record Python, `arduino-cli`, board-core versions, commit SHA, and
the compile log.

## Stage 1 — powered bench, no props, ESC supply isolated

1. Flash the recorded artifact. Confirm serial boot has no task-creation, MPU, BME280, or GPS-registration
   errors. Confirm the `samm10q` peripheral reports initializer then loop callbacks in the correct roles.
2. Join `KAF_Quadcopter_Drone`; confirm client address/gateway are both in `192.168.1.0/24`, ESP32 is
   `192.168.1.240`, and TCP port 70 accepts a connection.
3. Request telemetry 100 times at 20 Hz. Require exactly 64 payload bytes, protocol version 1, finite
   position/velocity/angular-rate fields, normalized quaternion (`abs(norm-1)<1e-3`) whenever attitude-valid
   is set, echoed message IDs, and no malformed/coalesced responses. Require battery-valid clear and ignore
   the placeholder battery float.
4. Before GPS is valid, origin request must return failure. Spoof sender `G`: origin and Pi-setpoint
   commands must return failure. Send NaN/Inf and a 31/33-byte setpoint: each must fail without changing
   telemetry sequence, mode, flags, or target.
5. TCP framing: send one valid 36-byte Pi-setpoint request as fragments of 1, 3, 13, 18, and 1 bytes with
   a loop interval between fragments. Require no early reply and one success only after byte 36. Then write a
   36-byte setpoint plus a 4-byte telemetry request in one TCP write. Require two ordered replies on successive
   processing iterations (success, then telemetry); neither request may absorb bytes from the other. Disconnect
   midway through a setpoint, reconnect, and require the next complete request to parse from a clean header.
6. From sender `X`, send otherwise-valid disarm, arm, flight-mode, SET_INFO, SET_STEST, SET_SETPT,
   SET_MOTORS, SET_CALIB, and SET_KAFENV requests. Require failure and an unchanged full state snapshot.
   Verify `P` and `G` can still disarm. Do not live-probe SET_INVOKEFUNC/SET_MEMORY with arbitrary pointers;
   their sender guards are checked by the host contract. On the no-prop bench, separately confirm the
   intentionally universal COM_SET_KILL policy and record the restart.

Pass: 100/100 valid telemetry replies; every negative command fails and leaves state unchanged.

## Stage 2 — live sensors, no props

1. Outdoors, wait for at least 6 satellites and HDOP <= 3.0. Keep disarmed. Send origin request from `P`;
   require success, position/velocity near zero, `origin_set=1`, `position_valid=1`, accepted sequence 0,
   Pi-stream/fresh flags clear. Log the next ten Z updates: they must remain launch-relative near zero, not
   jump to the old barometer reference. If the barometer was not fresh at latch, require GPS-relative Z and
   ensure a later un-rebased barometer sample is not adopted.
2. Send setpoint sequence 1 from `P`; require success and telemetry sequence 1 with stream/fresh flags set.
   Repeat 1 and then send 0; both must fail without altering sequence. Send 2; it must succeed.
3. Restart only the Pi. Sequence 1 must fail until the disarmed origin command succeeds again; after that,
   sequence 1 must succeed. This proves deliberate recovery without allowing in-flight replay.
4. Stop GPS data for >2 s. Position-valid must clear. Position-mode selection, arming, and new Pi setpoints
   must fail. Restore GPS and require validity to recover before continuing.
5. While disarmed, send Pi LAND (`0x5b`, payload `0x02`): require success but keep actuating flag clear.
   Send any other legacy path from `P`: require failure. From an already actuating position-mode test, Pi LAND
   must retain actuation and select the descent trajectory rather than cutting motors.

Pass: every acceptance/rejection and flag transition matches the contract, with logged timestamps.

## Stage 3 — control/failsafe bench, props removed, ESCs powered and restrained

1. With default `hoverThrust=0`, send a valid Pi target, select mode `0x0e`, then arm from `P`. Require
   failure, actuating clear, and control-calibrated clear. Repeat with hover thrust exactly 1, invalid gravity,
   and an invalid/non-finite critical PID; each must fail closed. Then load only a calibration measured and
   reviewed for this airframe (complete Stage 4 first if none exists), require control-calibrated set, arm,
   and maintain 20 Hz monotonically increasing setpoints for 10 s; mode must remain position mode.
2. Stop the stream. Within 500 ms plus one commander-loop interval, require transition to controlled descent
   (`CMD_DESCENT_MODE|TRAJECTORY_MODE`) or disarm if a finite emergency path cannot be formed. Confirm no
   stale command is accepted afterward.
3. Repeat while streaming, then disconnect the phone/ground station but retain Pi. The Pi flight must not
   land solely because entity `G` disappeared. Disconnect Pi instead: failsafe must trigger.
4. Select a legacy inline 22-float trajectory from `G`. Wait >500 ms. It must not trigger the Pi heartbeat
   watchdog; disconnecting `G` or invalidating position still must trigger the existing safety response.
5. Exercise circle path with props removed. Immediately after selection, setpoint index 0 must start at zero
   and rise with flight time; Y t-squared coefficient is index 9. No NaN/Inf motor command is permitted.

Pass: all deadlines/transitions occur and motors go/remain zero after disarm. Save telemetry plus serial log.

## Stage 4 — tethered low-energy propulsion

Use a rated test stand/cage, eye protection, remote kill, a trained safety observer, and a written abort
procedure. First run the existing per-motor direction test. Calibrate hover feed-forward for this airframe;
the firmware default is intentionally zero. Verify thrust saturation, motor mapping, kill, inverted-attitude,
GPS-loss, Pi-loss, and phone-loss behavior one at a time. Do not combine faults. Low-battery behavior cannot
be credited until a range-checked, timestamped battery source exists; the placeholder and clear battery-valid
flag are an explicit release blocker for that failsafe.

Pass criteria must be chosen and signed off by the flight-test lead before energizing motors (maximum thrust,
response time, allowable vibration, position error, and containment loads). A software-only result cannot
set safe physical thresholds.

## Stage 5 — flight progression

Proceed only after stages 0–4 have complete artifacts and safety sign-off: restrained hover, enclosed hover,
manual abort drill, stationary position hold, slow waypoint, then mission trajectory. At each stage test only
one new behavior, keep geofence/altitude/speed conservative, and confirm every autonomous fault transitions
to the documented landing/disarm state. Do not call the stack flight-ready until the real-airframe logs meet
the team-approved numeric criteria.
