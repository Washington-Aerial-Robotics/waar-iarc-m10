"""Run each software suite separately; never connect to flight hardware.

Run from any directory: python scripts/check_software.py
Install requirements-test.txt first. ROS2, Flutter and Arduino checks are separate.
"""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
COMMANDS = [
    (ROOT, ["-m", "unittest", "discover", "-s", "ESP32/KAF_Drone/tests/vicon", "-p", "test_*.py", "-v"]),
    (ROOT / "Autonomy/waar_autonomy", ["-m", "pytest", "tests", "-q"]),
    (ROOT / "Autonomy/mas_coordinator", ["-m", "pytest", "tests", "-q"]),
]


def main():
    failed = False
    for cwd, args in COMMANDS:
        print(f"\nChecking {cwd.relative_to(ROOT) or '.'}: {' '.join(args)}", flush=True)
        result = subprocess.run([sys.executable, *args], cwd=cwd)
        failed |= result.returncode != 0
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
