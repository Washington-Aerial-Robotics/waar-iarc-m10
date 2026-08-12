"""
PID gain stepping helper for the built-in webserver's Calibration page.

This does NOT decide gain values or detect oscillation on its own - it only
sends/reads the same cmd?SCAL=/cmd?DCAL= HTTP endpoints the browser's
Calibration page uses, so you don't have to click into each field and type by
hand. You still do the physical nudge-and-watch judgment call between every
change, exactly as before; this just removes the manual typing.

kafenv.cal's flat float-array layout (index -> field), traced from
kaf_drone.h/kaf_reset() - keep this in sync if the struct ever changes:
    0        anglealpha
    1        positionalpha
    2        gravitation
    3-11     gyrofilt[0..2]      (gain, ofst, stdv each)
    12-20    accelfilt[0..2]
    21-29    magfilt[0..2]
    30-53    sensefilt[0..7]
    54-56    xpid    (Kp, Ki, Kd)   - Position, unused in Acceleration Control mode
    57-59    vpid    (Kp, Ki, Kd)   - Velocity, unused in Acceleration Control mode
    60-62    apid    (Kp, Ki, Kd)   - Thrust
    63-65    qpid    (Kp, Ki, Kd)   - Attitude
    66-68    wpid[0] (Kp, Ki, Kd)   - W Rate X (roll rate)
    69-71    wpid[1] (Kp, Ki, Kd)   - W Rate Y (pitch rate)
    72-74    wpid[2] (Kp, Ki, Kd)   - W Rate Z (yaw rate)
    75       hoverThrust                    - "Hover FF" on the Control Constants panel

Usage:
    python pid_calibrate.py get all
    python pid_calibrate.py get qpid
    python pid_calibrate.py set qpid 0.15 0 0
    python pid_calibrate.py set wpid0 0.05 0 0
    python pid_calibrate.py set hoverThrust 0.15

Optional --ip if the drone isn't at the default AP address:
    python pid_calibrate.py --ip 192.168.1.240 get qpid
"""

import argparse
import struct
import sys
import urllib.request

DEFAULT_IP = "192.168.1.240"

# name -> tuple of calibration indices, in (Kp, Ki, Kd) order for PID fields,
# or a single-element tuple for scalar fields.
FIELD_MAP = {
    "xpid":        (54, 55, 56),
    "vpid":        (57, 58, 59),
    "apid":        (60, 61, 62),
    "qpid":        (63, 64, 65),
    "wpid0":       (66, 67, 68),
    "wpid1":       (69, 70, 71),
    "wpid2":       (72, 73, 74),
    "hoverthrust": (75,),
}

TOTAL_FLOATS = 76  # sizeof(kafenv.cal) / sizeof(float) - matches cmd?DCAL='s payload length


def fetch_all(ip: str) -> list[float]:
    url = f"http://{ip}/cmd?DCAL="
    with urllib.request.urlopen(url, timeout=5) as resp:
        raw = resp.read()
    count = len(raw) // 4
    return list(struct.unpack(f"<{count}f", raw[: count * 4]))


def set_field(ip: str, index: int, value: float) -> None:
    url = f"http://{ip}/cmd?SCAL={index}${value}"
    with urllib.request.urlopen(url, timeout=5) as resp:
        resp.read()


def cmd_get(args: argparse.Namespace) -> None:
    values = fetch_all(args.ip)
    if len(values) != TOTAL_FLOATS:
        print(
            f"Warning: got {len(values)} floats back, expected {TOTAL_FLOATS} - "
            "firmware's kafenv.cal layout may not match this script's FIELD_MAP "
            "(re-check the index table at the top of this file).",
            file=sys.stderr,
        )

    if args.field == "all":
        for name, indices in FIELD_MAP.items():
            parts = [f"{values[i]:.4f}" if i < len(values) else "?" for i in indices]
            print(f"{name:12s} = {', '.join(parts)}")
        return

    key = args.field.lower()
    if key not in FIELD_MAP:
        print(f"Unknown field '{args.field}'. Known fields: {', '.join(FIELD_MAP)}", file=sys.stderr)
        sys.exit(1)
    indices = FIELD_MAP[key]
    parts = [f"{values[i]:.4f}" if i < len(values) else "?" for i in indices]
    print(f"{args.field} = {', '.join(parts)}")


def cmd_set(args: argparse.Namespace) -> None:
    key = args.field.lower()
    if key not in FIELD_MAP:
        print(f"Unknown field '{args.field}'. Known fields: {', '.join(FIELD_MAP)}", file=sys.stderr)
        sys.exit(1)

    indices = FIELD_MAP[key]
    if len(args.values) != len(indices):
        noun = "value" if len(indices) == 1 else "values (Kp, Ki, Kd)"
        print(f"'{args.field}' takes {len(indices)} {noun}, got {len(args.values)}.", file=sys.stderr)
        sys.exit(1)

    for index, value in zip(indices, args.values):
        set_field(args.ip, index, value)
    print(f"Set {args.field} = {', '.join(str(v) for v in args.values)}")
    print(
        "NOT saved to EEPROM yet - a reboot (crash/brownout/reset) will silently revert this "
        "to the kaf_reset() defaults (Kp=1 on every loop). Once you've confirmed this value is "
        "good, run: python pid_calibrate.py save"
    )


def cmd_save(args: argparse.Namespace) -> None:
    url = f"http://{args.ip}/cmd?CSAV="
    with urllib.request.urlopen(url, timeout=5) as resp:
        resp.read()
    print("Saved current calibration to EEPROM.")


def cmd_zero(args: argparse.Namespace) -> None:
    # CSX0 zeroes kafenv.state.x/v/q/w entirely - whatever orientation the
    # drone is physically held in at the moment this runs becomes the new
    # zero reference (including yaw, which otherwise only accumulates via
    # gyro integration and drifts further from zero the more you rotate it).
    url = f"http://{args.ip}/cmd?CSX0="
    with urllib.request.urlopen(url, timeout=5) as resp:
        resp.read()
    print("Zeroed state estimate (position/velocity/attitude/rate) - hold it level and still before doing this.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"Drone webserver IP (default {DEFAULT_IP})")
    sub = parser.add_subparsers(dest="action", required=True)

    p_get = sub.add_parser("get", help="Read current value(s) from the drone")
    p_get.add_argument("field", help="Field name (see FIELD_MAP), or 'all'")
    p_get.set_defaults(func=cmd_get)

    p_set = sub.add_parser("set", help="Write value(s) to the drone")
    p_set.add_argument("field", help="Field name (see FIELD_MAP)")
    p_set.add_argument("values", type=float, nargs="+", help="Kp Ki Kd (or a single value for scalar fields)")
    p_set.set_defaults(func=cmd_set)

    p_save = sub.add_parser("save", help="Persist current calibration to EEPROM (survives reboot/crash)")
    p_save.set_defaults(func=cmd_save)

    p_zero = sub.add_parser("zero", help="Zero position/velocity/attitude/rate estimate (hold level+still first)")
    p_zero.set_defaults(func=cmd_zero)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
