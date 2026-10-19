"""
Relay (Astrom-Hagglund) autotune of the KAF drone's body-rate PIDs, plus an IMU orientation check.
Meant for the 3-DOF gimbal test bed with every axis but the tuned one locked. See README_autotune.md.

    python relay_autotune.py check                    # IMU axes vs motor layout - run this FIRST
    python relay_autotune.py tune --axis roll         # then roll, pitch, yaw - one at a time
    python relay_autotune.py tune --axis yaw --relay 0.08

Everything goes to the drone at --ip (default 192.168.0.240, the lab-router address):
  - TCP port 70 (kaf_link.py) for the tune itself and live attitude/state
  - HTTP (same endpoints as the browser ground station) for flight mode, calibration fields and saving

Body frame the firmware's motor mixing needs (flight.cpp flight_attitudeControl + the motor layout verified
by motor_direction_test.py: M0 front-left, M1 rear-left, M2 front-right, M3 rear-right):
    x = FRONT, y = LEFT, z = UP      ->  axis 0 (w.x) = roll, axis 1 (w.y) = pitch, axis 2 (w.z) = yaw
`check` verifies the IMU actually reports in that frame before anything is tuned.

SAFETY: `tune` arms the ESCs and spins the props at hover throttle. Drone clamped in the gimbal, everyone
clear of the prop arc, browser Kill button open. Ctrl+C stops the motors (the drone also stops by itself
if this script stops polling for 1 s, or the axis passes --max-angle / --max-rate).
"""

import argparse
import math
import os
import struct
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "vicon"))
from kaf_link import (COM_REQUEST_RELAYTUNE, COM_REPLY_RELAYTUNE, COM_SET_PIDTUNING, RELAY_STATUS,  # noqa: E402
                      KafLink)
from pid_calibrate import fetch_all, set_field  # noqa: E402

DEFAULT_IP = "192.168.0.240"

TUNING_MODE_START_PHYS = 3
TUNING_MODE_EXIT = 9
# pid_tuner.cpp relaycommand: mode, axis, relay, hysteresis, throttle, maxAngle, maxRate, duration
RELAY_COMMAND = struct.Struct("<BB6f")

# flight.h DEFAULT_MODES_MASK values used through the webserver's SFLT
NULL_MODE, INACTIVE_MODE, CALIBRATION_MODE = 0, 1, 2

# kafenv.cal float indices (see pid_calibrate.py)
CAL_GRAVITATION = 2
CAL_WPID = {0: (66, 67, 68), 1: (69, 70, 71), 2: (72, 73, 74)}
CAL_HOVER_THRUST = 75

AXES = {"roll": 0, "pitch": 1, "yaw": 2}
AXIS_NAMES = {0: "roll (w.x)", 1: "pitch (w.y)", 2: "yaw (w.z)"}

PHASES = {0: "idle", 1: "arming", 2: "spin-up", 3: "relay", 4: "spin-down", 5: "DONE", 6: "ABORTED"}
ABORTS = {
    0: "",
    1: "axis tilted past --max-angle",
    2: "axis spun faster than --max-rate - if this happened within a fraction of a second of the relay "
       "starting, the torque sign is backwards (motor layout vs IMU axes): run `check`",
    3: "link lost (no status poll for 1 s)",
    4: "no clean oscillation - try a larger --relay (or smaller --hyst)",
    5: "oscillation kept growing - torque sign/motor mixing vs IMU axes mismatch (run `check`), or the rig "
       "isn't locking the other axes",
    6: "stopped from the laptop",
}


def http(ip, cmd):
    with urllib.request.urlopen(f"http://{ip}/cmd?{cmd}", timeout=5) as resp:
        return resp.read()


def set_flight_mode(ip, mode):
    http(ip, f"SFLT={mode}$")


def tilt_angles(state):
    """Degrees: rotation about body x, body y (from the q.x/q.y = sqrt(2)*sin(angle/2) terms) and yaw q.z."""
    def tilt(q):
        return math.degrees(2 * math.asin(max(-1.0, min(1.0, q / math.sqrt(2)))))
    return tilt(state.q[0]), tilt(state.q[1]), math.degrees(state.q[2])


def calibrate(ip, seconds=6.0):
    print(f"Sensor calibration: keep the drone LEVEL and STILL for {seconds:.0f} s ...")
    set_flight_mode(ip, CALIBRATION_MODE)
    time.sleep(seconds)
    set_flight_mode(ip, NULL_MODE)
    time.sleep(0.2)
    set_flight_mode(ip, INACTIVE_MODE)  # attitude estimation running, motors off
    time.sleep(0.5)
    gravitation = fetch_all(ip)[CAL_GRAVITATION]
    print(f"  calibrated gravitation = {gravitation:+.2f} m/s^2")
    return gravitation


def poll_state(link, period=0.05):
    link.request_state()
    time.sleep(period)
    link.poll()
    return link.latest_state


def record(link, seconds):
    """Samples (time, angles, rates) for `seconds` at ~20 Hz."""
    samples = []
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        state = poll_state(link)
        if state is not None:
            samples.append((time.perf_counter(), tilt_angles(state), tuple(math.degrees(w) for w in state.w)))
    return samples


def gesture(link, instruction):
    input(f"\n>>> {instruction}\n    Press Enter, then do it within 2 s and HOLD still until 'done'.")
    base = record(link, 0.6)
    moving = record(link, 4.0)
    hold = record(link, 3.0)
    print("    done.")
    if len(base) < 3 or len(hold) < 3:
        sys.exit("No state replies from the drone - is it connected and in Inactive mode?")
    avg = lambda s, k: sum(x[1][k] for x in s) / len(s)  # noqa: E731
    delta = [avg(hold[-10:], k) - avg(base, k) for k in range(3)]
    # gyro-integrated change over the same window, to compare against the accel/mag-fused angle
    gyro = [0.0, 0.0, 0.0]
    window = base[-1:] + moving + hold
    for (t0, _, _), (t1, _, w1) in zip(window, window[1:]):
        for k in range(3):
            gyro[k] += w1[k] * (t1 - t0)
    yaw_drift = hold[-1][1][2] - hold[0][1][2]
    return delta, gyro, yaw_drift


def dominant(delta, k_expected, sign_expected, label):
    k = max(range(3), key=lambda i: abs(delta[i]))
    sign = 1 if delta[k] > 0 else -1
    ok = k == k_expected and sign == sign_expected and abs(delta[k]) > 10
    axis = "xyz"[k]
    print(f"    {label}: biggest change on body {axis} = {delta[k]:+.0f} deg "
          f"(expected body {'xyz'[k_expected]} {'+' if sign_expected > 0 else '-'})  -> {'OK' if ok else 'WRONG'}")
    return ok, k, sign


def cmd_check(args):
    print(__doc__.split("SAFETY")[0])
    print("This check does NOT spin the motors. Hold the drone in your hands (props on or off).")
    gravitation = calibrate(args.ip)
    link = KafLink(args.ip)
    results = []
    if gravitation < 0:
        print("  FAIL: gravity reads NEGATIVE along body z - the MPU9250 is mounted the other way up from what\n"
              "        periph_mpu9250.cpp assumes (it negates x and z, i.e. expects the chip face-DOWN).")
        results.append(False)
    else:
        results.append(True)

    # physical motions -> expected body rotation for x=FRONT, y=LEFT, z=UP:
    #   nose up          = positive rotation about RIGHT = about -y
    #   right side down  = positive rotation about FRONT = about +x
    #   counter-clockwise (seen from above) = positive about UP = +z
    tests = [
        ("Tilt the NOSE (front) UP about 30 deg", 1, -1, "nose up"),
        ("Tilt the RIGHT side DOWN about 30 deg (back to level first)", 0, +1, "right side down"),
        ("Back to level, then ROTATE about 90 deg COUNTER-CLOCKWISE seen from above", 2, +1, "rotate CCW"),
    ]
    found = {}
    for instruction, k_exp, s_exp, label in tests:
        delta, gyro, yaw_drift = gesture(link, instruction)
        ok, k, sign = dominant(delta, k_exp, s_exp, label)
        found[label] = (k, sign)
        results.append(ok)
        if abs(gyro[k]) > 10 and (gyro[k] > 0) != (delta[k] > 0):
            print(f"    WARNING: gyro integrates {gyro[k]:+.0f} deg on body {'xyz'[k]} but the fused angle moved "
                  f"{delta[k]:+.0f} - gyro and accelerometer/magnetometer disagree in sign on this axis")
            results.append(False)
        if label == "rotate CCW":
            print(f"    yaw drift while holding still: {yaw_drift:+.1f} deg "
                  f"({'OK' if abs(yaw_drift) < 10 else 'magnetometer fights the gyro - mag axes/sign or calibration'})")
            results.append(abs(yaw_drift) < 10)
    link.close()

    print("\n================ RESULT ================")
    if all(results):
        print("PASS - IMU reports x=FRONT, y=LEFT, z=UP, matching the motor mixing. Safe to run `tune`.")
        print("Note for Vicon: yaw 0 means the FRONT points along world +x (body x), so boot/calibrate the drone\n"
              "facing Vicon +X (not +Y), or pass --yaw-offset-deg to vicon_link.py.")
        return
    print("FAIL - do NOT tune or fly until fixed.")
    # Express the physical axes in the current body axes, then print the remap periph_mpu9250.cpp needs.
    vec = {}
    if "right side down" in found:
        k, s = found["right side down"]; vec["FRONT"] = (k, s)   # noqa: E702
    if "nose up" in found:
        k, s = found["nose up"]; vec["LEFT"] = (k, -s)           # noqa: E702  (rotation about +right = -left)
    if "rotate CCW" in found:
        k, s = found["rotate CCW"]; vec["UP"] = (k, s)           # noqa: E702
    if len({v[0] for v in vec.values()}) == 3:
        print("Measured: physical directions in the CURRENT body axes:")
        for name, (k, s) in vec.items():
            print(f"    {name:5s} = {'+' if s > 0 else '-'}{'xyz'[k]}")
        print("Fix in periph_mpu9250.cpp: after the existing sign flips, remap accel, gyro AND mag together:")
        for new, name in zip("xyz", ("FRONT", "LEFT", "UP")):
            k, s = vec[name]
            print(f"    new.{new} = {'' if s > 0 else '-'}old.{'xyz'[k]}")
        print("Then reflash and re-run `check`. Send me this output if unsure.")
    else:
        print("The motions didn't map to three distinct axes - repeat more slowly/cleanly, and send me the output.")


def read_status(link):
    payload = link.replies.get(COM_REPLY_RELAYTUNE)
    if payload is None:
        return None
    phase, axis, cycles, reason, *f = RELAY_STATUS.unpack(payload)
    keys = ("elapsed", "rate", "angle", "output", "amplitude", "period", "Ku", "Kp", "Ki", "Kd")
    return dict(phase=phase, axis=axis, cycles=cycles, reason=reason, **dict(zip(keys, f)))


def stop(link, ip):
    for _ in range(3):
        try:
            link.send(COM_SET_PIDTUNING, bytes([TUNING_MODE_EXIT]))
        except OSError:
            break
        time.sleep(0.05)
    try:
        http(ip, "SARM=0")
    except OSError:
        pass


def cmd_tune(args):
    axis = AXES[args.axis]
    cal = fetch_all(args.ip)
    throttle = args.throttle if args.throttle is not None else cal[CAL_HOVER_THRUST]
    if throttle <= 0:
        sys.exit("hoverThrust is 0 on the drone - pass --throttle 0.15 (or set it with pid_calibrate.py)")
    max_angle = args.max_angle if args.max_angle is not None else (120.0 if axis == 2 else 30.0)
    print(f"Axis {AXIS_NAMES[axis]}  relay {args.relay}  hysteresis {args.hyst} rad/s  throttle {throttle:.3f}  "
          f"limits {max_angle:.0f} deg / {args.max_rate:.0f} deg/s  duration {args.duration:.0f} s")
    print("Current gains:", [round(cal[i], 4) for i in CAL_WPID[axis]], "(Kp, Ki, Kd)")
    print("\nBEFORE CONTINUING: drone clamped in the gimbal with ONLY this axis free, props on, area clear,")
    print("browser Kill button open. Ctrl+C at any time stops the motors.")
    if input("Type 'yes' to start: ").strip().lower() != "yes":
        sys.exit("Not started.")

    if not args.no_calibrate:
        if calibrate(args.ip) < 0:
            sys.exit("Gravity reads negative - IMU orientation problem. Run `check` first.")
    link = KafLink(args.ip)
    try:
        link.send(COM_SET_PIDTUNING, RELAY_COMMAND.pack(
            TUNING_MODE_START_PHYS, axis, args.relay, args.hyst, throttle,
            math.radians(max_angle), math.radians(args.max_rate), args.duration))
        time.sleep(0.1)
        status, last_print = None, 0.0
        while True:
            link.send(COM_REQUEST_RELAYTUNE)  # also the drone's 1 s link watchdog
            time.sleep(0.05)
            link.poll()
            status = read_status(link)
            if status is None:
                continue
            now = time.perf_counter()
            if now - last_print > 0.25:
                last_print = now
                print(f"  {PHASES.get(status['phase'], '?'):9s} t={status['elapsed']:5.1f}s  "
                      f"angle {math.degrees(status['angle']):+6.1f} deg  rate {math.degrees(status['rate']):+7.1f} deg/s  "
                      f"out {status['output']:+.3f}  cycles {status['cycles']}")
            if status["phase"] in (5, 6):
                break
    except KeyboardInterrupt:
        print("\nStopping motors ...")
        stop(link, args.ip)
        link.close()
        sys.exit("Stopped.")
    except Exception:
        stop(link, args.ip)
        raise
    link.close()

    if status["phase"] == 6:
        sys.exit(f"ABORTED: {ABORTS.get(status['reason'], status['reason'])}")
    print("\n================ RESULT ================")
    print(f"oscillation: amplitude {math.degrees(status['amplitude']):.1f} deg/s, period Tu {status['period']:.3f} s "
          f"({status['cycles']} cycles)")
    print(f"ultimate gain Ku = {status['Ku']:.4f}")
    print(f"Tyreus-Luyben gains for {AXIS_NAMES[axis]}:  Kp {status['Kp']:.4f}  Ki {status['Ki']:.4f}  "
          f"Kd {status['Kd']:.5f}")
    print("current gains:", [round(cal[i], 4) for i in CAL_WPID[axis]])
    if input("Apply these gains and save to the drone's flash? [y/N]: ").strip().lower() != "y":
        print("Not applied.")
        return
    for index, value in zip(CAL_WPID[axis], (status["Kp"], status["Ki"], status["Kd"])):
        set_field(args.ip, index, value)
    http(args.ip, "CSAV=")
    check = fetch_all(args.ip)
    print("Saved. Drone now reports:", [round(check[i], 4) for i in CAL_WPID[axis]])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=DEFAULT_IP)
    sub = parser.add_subparsers(dest="command", required=True)
    p_check = sub.add_parser("check", help="verify IMU axes match the motor layout (no motors)")
    p_check.set_defaults(func=cmd_check)
    p_tune = sub.add_parser("tune", help="relay-autotune one rate axis on the gimbal (spins motors)")
    p_tune.add_argument("--axis", choices=AXES, required=True)
    p_tune.add_argument("--relay", type=float, default=0.03,
                        help="relay torque, mixer units (default 0.03; yaw needs more, try 0.08)")
    p_tune.add_argument("--hyst", type=float, default=0.05, help="switching band, rad/s (default 0.05)")
    p_tune.add_argument("--throttle", type=float, help="collective, hoverThrust units (default: drone's hoverThrust)")
    p_tune.add_argument("--max-angle", type=float, help="abort angle, deg (default 30, yaw 120)")
    p_tune.add_argument("--max-rate", type=float, default=200.0, help="abort rate, deg/s (default 200)")
    p_tune.add_argument("--duration", type=float, default=20.0, help="max relay time, s (default 20)")
    p_tune.add_argument("--no-calibrate", action="store_true", help="skip the sensor calibration before tuning")
    p_tune.set_defaults(func=cmd_tune)
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
