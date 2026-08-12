"""
One-shot capture of the full cmd?GFLT= response - every field (armed, mode,
step, x, r, t, a, w), not just the trimmed w/t line axis_motor_monitor.py
prints. Use this for point-in-time diagnostic captures (e.g. "what is the
controller doing right now, at rest, before any disturbance").

Usage:
    python gflt_snapshot.py
    python gflt_snapshot.py --ip 192.168.1.240
    python gflt_snapshot.py --raw   # print unformatted JSON only
"""

import argparse
import json
import urllib.request

DEFAULT_IP = "192.168.1.240"

FIELD_NOTES = {
    "armed": "1 = actuation enabled",
    "mode": "current DEFAULT_MODES_MASK flight mode index",
    "step": "last flight loop step time (ms)",
    "x": "estimated position [x, y, z] (m)",
    "r": "attitude rotation matrix (9 values, row-major)",
    "a": "world-frame estimated acceleration [x, y, z] (m/s2)",
    "w": "body-frame angular rate [roll, pitch, yaw] (rad/s)",
    "t": "motor commands [M0, M1, M2, M3] (post-mix, post-sqrt, 0-1)",
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"Drone IP (default {DEFAULT_IP})")
    parser.add_argument("--raw", action="store_true", help="Print unformatted JSON only")
    args = parser.parse_args()

    url = f"http://{args.ip}/cmd?GFLT="
    with urllib.request.urlopen(url, timeout=5) as resp:
        data = json.loads(resp.read())

    if args.raw:
        print(json.dumps(data))
        return

    print("Raw cmd?GFLT= snapshot:\n")
    for key, note in FIELD_NOTES.items():
        value = data.get(key, "<missing>")
        print(f"{key:6s} = {value}")
        print(f"         ({note})\n")

    extra = set(data) - set(FIELD_NOTES)
    for key in extra:
        print(f"{key:6s} = {data[key]}  (unlisted field)")


if __name__ == "__main__":
    main()
