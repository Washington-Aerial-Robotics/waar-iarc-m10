#!/usr/bin/env bash
# Run the mine detection pipeline on Raspberry Pi (headless).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SLAM_DIR="$REPO_ROOT/SLAM"
VENV_DIR="$SLAM_DIR/.venv"

cd "$SLAM_DIR"

if [[ ! -d "$VENV_DIR" ]]; then
  python3 -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"
pip install --upgrade pip
pip install -r requirements.txt

if [[ ! -f "$REPO_ROOT/apriltags/camera_calib.npz" ]]; then
  echo "ERROR: missing apriltags/camera_calib.npz"
  echo "Run calibrate_camera.py on the Pi USB camera first."
  exit 1
fi

CONFIG="${1:-pipeline_config.pi.json}"
echo "Using config: $CONFIG"

EXTRA_ARGS=()
if [[ -n "${ESP32_HOST:-}" ]]; then
  EXTRA_ARGS+=(--esp32-host "$ESP32_HOST")
fi

python mine_detection_pipeline.py --config "$CONFIG" "${EXTRA_ARGS[@]}"
