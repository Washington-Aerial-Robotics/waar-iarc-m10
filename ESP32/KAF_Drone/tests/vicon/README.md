# Vicon Motion Capture Testing

For an **onboard Raspberry Pi 4/5 companion** (PC SDK relay, Pi installation,
physical test order and SD-card image capture), start with the
[single-drone Pi guide](../../../../deploy/raspberry-pi/README.md).
Use `relay --pi-ip <pi-ip>` on the PC and `monitor`/`bridge --udp-source <pc-ip>`
on the Pi; the Pi does not need the Vicon SDK.

Fly the KAF drone indoors using Vicon Tracker (tested against Tracker 3.10) for position, instead of GPS and the barometer.

```
Vicon PC (Windows, Tracker)  --Ethernet-->  your laptop  --WiFi, TCP 70-->  drone (firmware_vicon)
DataStream server, port 801                 vicon_link.py                   mocap_step() position source
```

Any laptop works. It needs to reach both the Vicon PC and the drone: easiest is one lab router for all three (section 2). Attitude and yaw still come from the IMU and magnetometer. Vicon replaces only the **position** source, which feeds the same IMU fusion GPS used (`flight.cpp` `positionEstimate()`).

## Files

| File | What it is |
|---|---|
| `vicon_link.py` | Laptop CLI: `vicon`, `monitor`, `bridge` modes |
| `kaf_link.py` | Drone TCP protocol (header + packed structs, port 70) |
| `../../soft/firmware_vicon/` | `firmware_full` with `mocap_step` as the position source |
| `../../src/auxilary/mocap.*` | `COM_SET_MOCAP` handler, staleness check, estimator |

## 1. Vicon PC (Tracker 3.10)

1. **Calibrate** the cameras with the wand, then **set the origin** with the wand on the floor. Wand axes become the world axes (Z up).
2. **Markers:** at least 4 reflective markers per drone, in an asymmetric pattern that's different on every drone. Tape over LEDs and shiny parts. In the camera view, check that nothing on the drone shows up as a marker except the markers themselves.
3. **Create an object** per drone (`waar1`, `waar2`, ...). Before you create it, place the drone on the floor **with its front facing world +X**. Tracker uses the pose at creation as the object's zero orientation, so Vicon yaw 0 then matches the firmware's yaw 0 (body x = front, see `../relay_autotune.py check`).
4. DataStream is on by default on **port 801**. Allow port 801 through Windows Firewall, then note the PC's IP on the Ethernet link to your laptop.

## 2. Network: lab router (recommended)

Put the Vicon PC, your laptop and the drone on one WiFi router, so everyone talks over the same `192.168.0.x` network:

```
                  TP-Link router 192.168.0.1
     ┌──────────────────┼────────────────────┐
 Vicon PC           your laptop          drone 192.168.0.240
 (Ethernet or WiFi)  (2.4 or 5 GHz)      (2.4 GHz only)
```

1. **Drone:** copy `src/firmware/wifi_credentials.h.example` to `wifi_credentials.h` (gitignored) and fill in the router's **2.4 GHz** SSID and password. The ESP32 cannot see 5 GHz networks: on a TP-Link, use the name *without* `_5G`. Flash `firmware_vicon` and the drone joins the router at the fixed IP from that file (`192.168.0.240`). Serial prints `GG>G[ip=192.168.0.240]` when it's on.
   - If it can't join within 20 s (wrong password, router off), it falls back to its own `KAF_Quadcopter_Drone` AP at `192.168.1.240`, so it's never unreachable. Serial prints `Falling Back To AP`.
   - Delete `wifi_credentials.h` to go back to AP-only behaviour.
2. **Vicon PC:** connect it to the router (Ethernet to a LAN port is best; WiFi works). Leave the camera network adapter alone. Note its `192.168.0.x` IP (`ipconfig`).
3. **Laptop:** join the router (either band is fine). Check with `ping 192.168.0.240` and `ping <vicon-ip>`.
4. **Several drones:** give each its own IP in its `wifi_credentials.h` (`.240`, `.241`, ...) and run one `bridge` per drone with its own `--drone-ip`/`--object`.
5. **University WiFi won't work:** eduroam-style networks block device-to-device traffic and need enterprise login. Use a dedicated router.

Without a router: Ethernet from laptop to Vicon PC, and laptop WiFi on the drone's `KAF_Quadcopter_Drone` AP (`192.168.1.240`). That's the default `--drone-ip`.

## 3. Laptop software

1. **Vicon DataStream SDK:** download it from vicon.com (free account), install it, then `pip install` the Python wheel/folder it ships (`vicon_dssdk`). Check with `python -c "from vicon_dssdk import ViconDataStream"`.
2. Disconnect the Flutter ground station app first. The drone accepts **one** TCP client on port 70. The browser ground station (`http://192.168.0.240`) still works alongside.
3. With the router setup, add `--drone-ip 192.168.0.240` to every `monitor`/`bridge` command below.

## 4. Bring-up, in order

**a. Vicon only** (no drone needed):
```
python vicon_link.py vicon --vicon <vicon-ip> --object waar1
```
Move the drone by hand. +X, +Y and +Z should match the directions you set with the wand.

**b. Monitor** (works with **any** firmware, sends nothing but state requests):
```
python vicon_link.py monitor --vicon <vicon-ip> --object waar1 --log monitor.csv
```
Boot the drone **facing Vicon +X** (yaw 0 is wherever the drone points when it calibrates, and body x is the front). Then rotate it by hand:
- `yaw diff` should stay near 0. If it's constant but not 0, you booted at an angle: pass that value as `--yaw-offset-deg` later, or reboot facing +X.
- If `yaw diff` drifts or jumps as you walk around, the magnetometer is disturbed indoors (steel, motors). Position control will then push in a rotated direction. Fix this before flying.

**c. Flash `firmware_vicon`** (`soft/firmware_vicon/firmware_vicon.ino`, same as flashing `firmware_full`). PID gains are kept: the EEPROM layout is unchanged.

**d. Bridge, props OFF:**
```
python vicon_link.py bridge --vicon <vicon-ip> --object waar1 --log bridge.csv
```
- `ok` should climb with `sent`. A `WARNING ... firmware_vicon?` means the drone is still on `firmware_full`.
- The drone's position should now follow Vicon to within a few cm as you carry it.
- Stop the script: within 150 ms the drone reports position invalid, so `POS_SETPOINT_MODE` / `TRAJECTORY_MODE` are refused (or, if flying, the commander failsafe lands it).

**e. Tethered/netted hover, then position hold, then square/qualification tests.** Keep the kill switch ready.

Test without the lab: `python vicon_link.py bridge --fake` sends a synthetic 0.5 m circle.

## 5. Things to know

- **Dropouts:** occluded frames are not sent. The drone trusts a position for `MOCAP_STALE_MS` (150 ms, `mocap.h`); after that `estimation_positionValid()` is false. That refuses position modes, and in flight it triggers the existing landing failsafe in `commander_step()`. In manual/accel modes, losing Vicon changes nothing.
- **Descent after dropout** finishes when `state.x.z < 0.1`, which is dead-reckoned once Vicon is gone. Don't rely on it as anything more than a last resort.
- **Fusion gain:** `kafenv.cal.positionalpha` was tuned around slow, noisy GPS. Vicon is fast and precise, so it can likely be raised. Tune it on the Calibration page while watching `bridge.csv`.
- **Yaw:** Vicon yaw is sent (`mocappose.yaw`) and stored (`mocap_yaw()`), but not fused yet. Feeding it into `kafenv.state.q.z` is the next step if the magnetometer is unreliable in the lab.
- **Several drones:** see section 2 - one router, one fixed IP per drone, one `bridge` per drone. Each drone also needs a unique `deviceID` (pass it with `--drone-id`).
