#!/usr/bin/env bash
# Calibrate the Pi USB camera at the resolution used in flight.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
APRILTAGS_DIR="$REPO_ROOT/apriltags"
VENV_DIR="$REPO_ROOT/SLAM/.venv"

CAMERA_INDEX="${CAMERA_INDEX:-0}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"

cd "$APRILTAGS_DIR"

if [[ -d "$VENV_DIR" ]]; then
  source "$VENV_DIR/bin/activate"
fi

echo "Calibrating camera index $CAMERA_INDEX at ${WIDTH}x${HEIGHT}"
echo "Edit CAMERA_INDEX / REQUEST_WIDTH / REQUEST_HEIGHT in calibrate_camera.py if needed."
echo "Output goes to apriltags/camera_calib.npz"
echo ""

python3 calibrate_camera.py

if [[ -f camera_calib.npz ]]; then
  echo ""
  echo "Done. camera_calib.npz is ready for the pipeline."
else
  echo "ERROR: calibration file not created"
  exit 1
fi
