
"""
Live side-by-side readout of gyro (w) and motor commands (t) so you can
directly correlate a physical rotation with the actual motor response,
instead of inferring it from watching/listening to the motors.

Polls the webserver's cmd?GFLT= endpoint (same one the homepage's attitude
graph uses) at ~6Hz and prints one continuously-updating line:

    w=[roll_rate, pitch_rate, yaw_rate]  t=[M0, M1, M2, M3]

How to use this to diagnose an axis/mixing problem:
  1. Drone armed, in Acceleration Control mode, with real wpid gains set
     (same state you've already been testing in).
  2. Run this script.
  3. Rotate ONLY in roll, watch which w component moves and which two
     motors change together. Expect: w.x (first value) moves, M0/M1 move
     one way and M2/M3 the other (roll pairing from flight.cpp's mixing:
     {M0,M1} vs {M2,M3}).
  4. Stop, rotate ONLY in pitch. Expect: w.y (second value) moves, M0/M2
     move one way and M1/M3 the other ({M0,M2} vs {M1,M3}).
  5. Stop, rotate ONLY in yaw. Expect: w.z (third value) moves, M0/M3 move
     one way and M1/M2 the other ({M0,M3} vs {M1,M2}).

If a different w component moves than the one you'd expect for the axis
you're rotating (e.g. rotating in pitch but w.x/roll-rate is what changes),
that's a real IMU axis-mapping mismatch, not a PID gain problem - no amount
of gain tuning fixes a mislabeled axis. If the correct w component moves but
the WRONG motor pairing reacts (e.g. M0/M2 change instead of M0/M1 for a
roll rotation), that points to a motor-mixing/wiring mismatch instead.
Ctrl+C to stop.

Usage:
    python axis_motor_monitor.py
    python axis_motor_monitor.py --ip 192.168.1.240 --rate 6
"""

import argparse
import json
import time
import urllib.request

DEFAULT_IP = "192.168.1.240"


def fetch_gflt(ip: str) -> dict:
    url = f"http://{ip}/cmd?GFLT="
    with urllib.request.urlopen(url, timeout=2) as resp:
        return json.loads(resp.read())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=DEFAULT_IP, help=f"Drone IP (default {DEFAULT_IP})")
    parser.add_argument("--rate", type=float, default=6.0, help="Poll rate in Hz (default 6)")
    args = parser.parse_args()

    print(__doc__)
    period = 1.0 / args.rate
    print("Polling... (Ctrl+C to stop)\n")
    start_time = time.monotonic()

    try:
        while True:
            start = time.monotonic()
            try:
                data = fetch_gflt(args.ip)
            except Exception as exc:  # noqa: BLE001 - just report and keep polling
                print(f"[fetch error: {exc}]")
                time.sleep(period)
                continue

            w = data.get("w", [0, 0, 0])
            t = data.get("t", [0, 0, 0, 0])
            armed = data.get("armed", 0)
            mode = data.get("mode", 0)

            wstr = ",".join(f"{v:+.3f}" for v in w)
            tstr = ",".join(f"{v:.2f}" for v in t)
            elapsed_total = time.monotonic() - start_time
            print(f"[{elapsed_total:6.2f}s] a={armed} m={mode} w=[{wstr}] t=[{tstr}]")

            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
