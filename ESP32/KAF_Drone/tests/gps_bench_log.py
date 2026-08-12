"""
Standalone SAM-M10Q GPS bench test - logs a coordinate every time you press
Enter (with 'a' typed, or just Enter - see below), independent of the ESP32
firmware entirely.

WHY STANDALONE: this validates the GPS chip itself and the exact local-frame
coordinate math this session fixed in periph_samm10q.cpp (longitude-string
bug, missing degrees->radians conversion, km/m scale error, x/y axis swap),
without needing to flash/debug through the drone. Wire the SAM-M10Q directly
to a USB-to-TTL serial adapter on this PC (not through the ESP32) - GPS
modules are 3.3V TTL UART, NOT true RS-232, so use a 3.3V-logic adapter, and
connect module TX -> adapter RX, module RX -> adapter TX, GND -> GND.

WHAT IT DOES:
  - Reads raw NMEA $GNGGA (falls back to $GPGGA) sentences in a background
    thread, continuously updating the latest fix.
  - Each time you press Enter at the prompt, logs the CURRENT latest fix -
    both the raw lat/lon/altitude/quality/satellites/HDOP, and a local x/y/z
    in metres relative to the FIRST point you log (that first point becomes
    the origin, same "first good fix = origin" idea as gps_setOrigin() in
    the firmware).
  - The local-frame conversion is a deliberate line-for-line copy of the
    FIXED firmware math (periph_samm10q.cpp): METERS_PER_DEGREE=111111.0
    (not the original buggy 111.111 km/degree), latitude converted to
    radians before cos(), longitude(+east)->x, latitude(+north)->y - so a
    working result here is real evidence the firmware-side fix is correct,
    not just that this separate script happens to work.
  - Applies the same quality gate the firmware uses (GPS_MIN_SATELLITES=6,
    GPS_MAX_HDOP=3.0) and warns (but still logs) if the point you're about
    to log doesn't meet it - useful to know if a logged point should be
    trusted the same way the firmware would trust it.
  - Writes every logged point to a CSV file for later comparison against
    tape-measured/known reference distances.

USAGE:
    python gps_bench_log.py --port COM5
    python gps_bench_log.py --port COM5 --baud 9600 --out my_gps_test.csv

Find your port in Windows Device Manager under "Ports (COM & LPT)" once the
USB-serial adapter is plugged in.

At the prompt: press Enter alone (or type anything + Enter) to log a point,
or type 'q' + Enter to quit.
"""

import argparse
import csv
import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import serial

# Line-for-line copy of the FIXED constants/formula in periph_samm10q.cpp -
# keep in sync if that file's math ever changes again.
METERS_PER_DEGREE = 111111.0
DEG2RAD = math.pi / 180.0
GPS_MIN_SATELLITES = 6
GPS_MAX_HDOP = 3.0


@dataclass
class Fix:
    latitude: float
    longitude: float
    altitude: float
    fix_quality: int
    satellites: int
    hdop: float
    received_at: float  # time.monotonic()


def parse_gga(line: str) -> Optional[Fix]:
    if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
        return None
    fields = line.split(",")
    if len(fields) < 10:
        return None
    try:
        lat_raw, lat_dir = fields[2], fields[3]
        lng_raw, lng_dir = fields[4], fields[5]
        fix_quality = int(fields[6]) if fields[6] else 0
        satellites = int(fields[7]) if fields[7] else 0
        hdop = float(fields[8]) if fields[8] else 0.0
        alt_field = fields[9].split("*")[0]
        altitude = float(alt_field) if alt_field else 0.0

        if not lat_raw or not lng_raw or fix_quality == 0:
            return None

        # ddmm.mmmm -> decimal degrees (same split the firmware uses: first
        # 2 digits of lat / first 3 of lon are whole degrees, rest is minutes/60).
        latitude = float(lat_raw[:2]) + float(lat_raw[2:]) / 60.0
        if lat_dir == "S":
            latitude = -latitude
        longitude = float(lng_raw[:3]) + float(lng_raw[3:]) / 60.0
        if lng_dir == "W":
            longitude = -longitude

        return Fix(latitude, longitude, altitude, fix_quality, satellites, hdop, time.monotonic())
    except (ValueError, IndexError):
        return None


def is_good_quality(fix: Fix) -> bool:
    return fix.fix_quality != 0 and fix.satellites >= GPS_MIN_SATELLITES and 0 < fix.hdop <= GPS_MAX_HDOP


class GpsReader:
    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.latest: Optional[Fix] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)
        self.ser.close()

    def get_latest(self) -> Optional[Fix]:
        with self._lock:
            return self.latest

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline().decode("ascii", errors="replace").strip()
            except Exception as exc:  # noqa: BLE001 - keep the reader alive
                print(f"[reader error: {exc}]")
                time.sleep(0.5)
                continue
            if not raw:
                continue
            fix = parse_gga(raw)
            if fix is not None:
                with self._lock:
                    self.latest = fix


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="Serial port the USB-TTL adapter shows up as, e.g. COM5")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default 9600, standard NMEA default)")
    parser.add_argument("--out", default=None, help="CSV output file (default: gps_bench_<timestamp>.csv)")
    args = parser.parse_args()

    out_path = args.out or f"gps_bench_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    print(__doc__)
    print(f"Opening {args.port} at {args.baud} baud...")
    reader = GpsReader(args.port, args.baud)
    reader.start()

    origin: Optional[Fix] = None
    point_count = 0

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "point", "timestamp", "latitude", "longitude", "altitude",
            "fix_quality", "satellites", "hdop", "good_quality",
            "local_x_m", "local_y_m", "local_z_m",
        ])

        print("Waiting for first fix...")
        try:
            while reader.get_latest() is None:
                time.sleep(0.5)
            print("First fix received. Press Enter to log a point, or 'q' + Enter to quit.\n")

            while True:
                cmd = input("> ").strip().lower()
                if cmd == "q":
                    break

                fix = reader.get_latest()
                if fix is None:
                    print("  No fix available yet - not logged.")
                    continue

                age = time.monotonic() - fix.received_at
                good = is_good_quality(fix)

                if origin is None:
                    origin = fix
                    local_x = local_y = local_z = 0.0
                    print("  This is the ORIGIN point (0, 0, 0) - all later points are relative to this one.")
                else:
                    local_x = METERS_PER_DEGREE * (fix.longitude - origin.longitude) * math.cos(fix.latitude * DEG2RAD)
                    local_y = METERS_PER_DEGREE * (fix.latitude - origin.latitude)
                    local_z = fix.altitude - origin.altitude

                point_count += 1
                print(
                    f"  Point {point_count}: lat={fix.latitude:.6f} lon={fix.longitude:.6f} alt={fix.altitude:.2f}m "
                    f"| sats={fix.satellites} hdop={fix.hdop:.2f} quality={fix.fix_quality} "
                    f"({'GOOD' if good else 'BELOW FIRMWARE THRESHOLD'}, fix age={age:.1f}s)"
                )
                print(f"  Local: x={local_x:.3f}m y={local_y:.3f}m z={local_z:.3f}m")
                if not good:
                    print(
                        f"  WARNING: fails the firmware's own gate (needs sats>={GPS_MIN_SATELLITES}, "
                        f"0<hdop<={GPS_MAX_HDOP}) - the real firmware would NOT have accepted this point."
                    )

                writer.writerow([
                    point_count, datetime.now().isoformat(), fix.latitude, fix.longitude, fix.altitude,
                    fix.fix_quality, fix.satellites, fix.hdop, good, local_x, local_y, local_z,
                ])
                f.flush()
        except KeyboardInterrupt:
            pass
        finally:
            reader.stop()

    print(f"\nLogged {point_count} point(s) to {out_path}")


if __name__ == "__main__":
    main()
