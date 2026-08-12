#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

ENV_FILE="${WAAR_ENV_FILE:-}"
HARDWARE=false
PROBE_ESP32=false
FAILURES=0
WARNINGS=0

usage() {
  cat <<'EOF'
Usage: preflight.sh [--env-file PATH] [--hardware] [--probe-esp32]

Read-only installation and safety checks. --hardware checks camera, route, and
optional serial devices. --probe-esp32 briefly opens the firmware TCP port and
is permitted only while the service is stopped, the vehicle is disarmed, props
are removed, and AIRFRAME_DISARMED_ACK=PROPS_REMOVED_AND_DISARMED is exported.
No service is started and no flight command is sent.
EOF
}

while (($#)); do
  case "$1" in
    --env-file)
      (($# >= 2)) || waar_die "--env-file needs a path"
      ENV_FILE="$2"
      shift 2
      ;;
    --hardware) HARDWARE=true; shift ;;
    --probe-esp32) HARDWARE=true; PROBE_ESP32=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) waar_die "unknown argument: $1" ;;
  esac
done

waar_load_env_file "$ENV_FILE"

REPO_DIR="${REPO_DIR:-}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
DRONE_ID="${DRONE_ID:-d1}"
DRY_RUN="${DRY_RUN:-true}"
AUTO_ARM="${AUTO_ARM:-false}"
HARDWARE_ENABLE_ACK="${HARDWARE_ENABLE_ACK:-disabled}"
ESP32_HOST="${ESP32_HOST:-192.168.1.240}"
ESP32_PORT="${ESP32_PORT:-70}"
TRANSPORT="${TRANSPORT:-tcp}"
SENDER_ID="${SENDER_ID:-P}"
ESP32_DEVICE_ID="${ESP32_DEVICE_ID:-U}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/v4l/by-id/REPLACE_WITH_STEREO_CAMERA}"
IMAGE_WIDTH="${IMAGE_WIDTH:-2560}"
IMAGE_HEIGHT="${IMAGE_HEIGHT:-960}"
FRAMERATE="${FRAMERATE:-10.0}"
BRIDGE_SERIAL_PORT="${BRIDGE_SERIAL_PORT:-}"
SENSOR_SERIAL_PORT="${SENSOR_SERIAL_PORT:-}"
ENABLE_PERCEPTION="${ENABLE_PERCEPTION:-true}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_link}"
ARENA_WIDTH_M="${ARENA_WIDTH_M:-91.44}"
ARENA_HEIGHT_M="${ARENA_HEIGHT_M:-24.38}"
ARENA_MAP_ALIGNED="${ARENA_MAP_ALIGNED:-false}"
APRILTAG_SIZE_M="${APRILTAG_SIZE_M:-REPLACE_WITH_MEASURED_BLACK_SQUARE_METERS}"
ARENA_FRAME_SURVEY_ID="${ARENA_FRAME_SURVEY_ID:-REPLACE_WITH_ARENA_FRAME_SURVEY_RECORD}"
CAMERA_EXTRINSICS_ID="${CAMERA_EXTRINSICS_ID:-REPLACE_WITH_CAMERA_EXTRINSICS_RECORD}"
STEREO_CALIBRATION_ID="${STEREO_CALIBRATION_ID:-REPLACE_WITH_STEREO_CALIBRATION_RECORD}"
YAW_ALIGNMENT_ID="${YAW_ALIGNMENT_ID:-REPLACE_WITH_YAW_ALIGNMENT_RECORD}"
BATTERY_SAFETY_ID="${BATTERY_SAFETY_ID:-REPLACE_WITH_INDEPENDENT_BATTERY_MONITOR_RECORD}"
MAX_FLIGHT_TIME_S="${MAX_FLIGHT_TIME_S:-REPLACE_WITH_CONSERVATIVE_TESTED_LIMIT}"
CONTROL_CALIBRATION_ID="${CONTROL_CALIBRATION_ID:-REPLACE_WITH_AIRCRAFT_CONTROL_CALIBRATION_RECORD}"
REQUIRE_BATTERY_VALID="${REQUIRE_BATTERY_VALID:-true}"
RUN_USER="${USER:-$(id -un)}"

pass() { printf '[PASS] %s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*" >&2; WARNINGS=$((WARNINGS + 1)); }
fail() { printf '[FAIL] %s\n' "$*" >&2; FAILURES=$((FAILURES + 1)); }

printf 'WAAR preflight (read-only)\n'
printf '  user=%s drone=%s dry_run=%s ros_domain=%s hardware_checks=%s\n' \
  "${USER:-unknown}" "$DRONE_ID" "$DRY_RUN" "$ROS_DOMAIN_ID" "$HARDWARE"

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]]; then
    pass "Ubuntu 22.04 detected"
  else
    fail "expected Ubuntu 22.04; found ${PRETTY_NAME:-unknown}"
  fi
else
  fail "/etc/os-release is missing"
fi

[[ "$ROS_DISTRO" == humble ]] && pass "ROS_DISTRO=humble" || fail "ROS_DISTRO must be humble"
if waar_is_uint "$ROS_DOMAIN_ID" && ((ROS_DOMAIN_ID <= 232)); then
  pass "ROS_DOMAIN_ID is valid"
else
  fail "ROS_DOMAIN_ID must be in 0..232"
fi
waar_is_bool "$DRY_RUN" && pass "DRY_RUN has a valid value" || fail "DRY_RUN must be true or false"
[[ "$AUTO_ARM" == false ]] && pass "AUTO_ARM is disabled" || fail "AUTO_ARM must remain false"
[[ "$TRANSPORT" == tcp ]] && pass "flight transport is TCP" || fail "TRANSPORT must be tcp"
[[ "$SENDER_ID" == P && "$ESP32_DEVICE_ID" == U ]] \
  && pass "firmware route is P -> U" \
  || fail "current firmware requires SENDER_ID=P and ESP32_DEVICE_ID=U"
frames_valid=true
for frame_name in MAP_FRAME ODOM_FRAME BASE_FRAME; do
  frame_value="${!frame_name}"
  if [[ ! "$frame_value" =~ ^[A-Za-z][A-Za-z0-9_/]*$ || "$frame_value" == /* ]]; then
    fail "$frame_name is not a valid non-empty relative frame ID"
    frames_valid=false
  fi
done
if [[ "$MAP_FRAME" == "$ODOM_FRAME" || "$MAP_FRAME" == "$BASE_FRAME" || "$ODOM_FRAME" == "$BASE_FRAME" ]]; then
  fail "MAP_FRAME, ODOM_FRAME, and BASE_FRAME must be distinct"
  frames_valid=false
fi
[[ "$frames_valid" == true ]] && pass "map/odom/base frame IDs are distinct and valid"
dimensions_valid=true
for dimension_name in ARENA_WIDTH_M ARENA_HEIGHT_M; do
  dimension_value="${!dimension_name}"
  if [[ ! "$dimension_value" =~ ^[0-9]+([.][0-9]+)?$ ]] \
      || ! awk -v value="$dimension_value" 'BEGIN {exit !(value > 0.0)}'; then
    fail "$dimension_name must be a positive surveyed dimension in metres"
    dimensions_valid=false
  fi
done
[[ "$dimensions_valid" == true ]] \
  && pass "arena dimensions are configured as ${ARENA_WIDTH_M}m x ${ARENA_HEIGHT_M}m"
[[ "$ARENA_MAP_ALIGNED" == true ]] \
  && pass "operator asserts the map is aligned to the surveyed arena frame" \
  || fail "ARENA_MAP_ALIGNED must be true only after the signed arena-frame survey"
if [[ "$APRILTAG_SIZE_M" != *REPLACE_* ]] \
    && [[ "$APRILTAG_SIZE_M" =~ ^0[.][0-9]+$ ]] \
    && awk -v size="$APRILTAG_SIZE_M" 'BEGIN {exit !(size > 0.0 && size < 1.0)}'; then
  pass "APRILTAG_SIZE_M is configured: ${APRILTAG_SIZE_M}m"
else
  fail "APRILTAG_SIZE_M must be replaced with the measured black-square size in metres"
fi
for record_name in \
    ARENA_FRAME_SURVEY_ID \
    CAMERA_EXTRINSICS_ID \
    STEREO_CALIBRATION_ID \
    YAW_ALIGNMENT_ID \
    BATTERY_SAFETY_ID \
    CONTROL_CALIBRATION_ID; do
  record_value="${!record_name}"
  if [[ -n "$record_value" && "$record_value" != *REPLACE_* ]]; then
    pass "$record_name references a release record"
  else
    fail "$record_name must reference the signed survey/calibration record"
  fi
done
if waar_is_uint "$MAX_FLIGHT_TIME_S" && ((MAX_FLIGHT_TIME_S >= 1)); then
  pass "MAX_FLIGHT_TIME_S is explicitly configured: ${MAX_FLIGHT_TIME_S}s"
else
  fail "MAX_FLIGHT_TIME_S must be an explicitly tested conservative positive limit"
fi
if [[ "$REQUIRE_BATTERY_VALID" == true ]]; then
  pass "bridge requires firmware BATTERY_VALID before arming"
elif [[ "$REQUIRE_BATTERY_VALID" == false \
    && "$BATTERY_SAFETY_ID" != *REPLACE_* \
    && "$MAX_FLIGHT_TIME_S" =~ ^[0-9]+$ \
    && "$MAX_FLIGHT_TIME_S" -ge 1 ]]; then
  warn "firmware battery validity bypass is configured; independent battery protection is mandatory"
else
  fail "REQUIRE_BATTERY_VALID must be true, or false only with independent battery protection records"
fi
if [[ "$DRY_RUN" == false ]]; then
  [[ "$HARDWARE_ENABLE_ACK" == I_ACCEPT_HARDWARE_OUTPUT ]] \
    && pass "hardware-output acknowledgement is present" \
    || fail "DRY_RUN=false without HARDWARE_ENABLE_ACK=I_ACCEPT_HARDWARE_OUTPUT"
fi

if [[ -n "$REPO_DIR" && -d "$REPO_DIR" ]]; then
  pass "repository directory exists: $REPO_DIR"
else
  fail "REPO_DIR is missing or not a directory: ${REPO_DIR:-<unset>}"
fi

if [[ -n "$REPO_DIR" ]]; then
  right_calibration="${REPO_DIR}/Autonomy/slam_package/slam/config/right.yaml"
  urdf_file="${REPO_DIR}/Autonomy/slam_package/slam/urdf/drone.urdf"
  if [[ -r "$right_calibration" && -r "$urdf_file" ]]; then
    if python3 - "$right_calibration" "$urdf_file" <<'PY'
import math
import re
import sys
from pathlib import Path

import yaml

right = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
p = right["projection_matrix"]["data"]
if len(p) != 12 or not math.isfinite(float(p[0])) or float(p[0]) == 0.0:
    raise SystemExit("invalid right projection matrix")
baseline = -float(p[3]) / float(p[0])
if not (0.001 < abs(baseline) < 1.0):
    raise SystemExit(f"implausible calibration baseline: {baseline}")

urdf = Path(sys.argv[2]).read_text(encoding="utf-8")
matches = re.findall(
    r'<joint\s+name="camera_(?:left|right)_joint".*?<origin\s+xyz="([^"]+)"',
    urdf,
    re.DOTALL,
)
if len(matches) != 2:
    raise SystemExit("could not locate both camera joint origins in URDF")
y = [float(value.split()[1]) for value in matches]
urdf_baseline = abs(y[1] - y[0])
tolerance_m = 0.002
if abs(abs(baseline) - urdf_baseline) > tolerance_m:
    raise SystemExit(
        f"baseline mismatch: right.yaml={abs(baseline):.6f}m "
        f"URDF={urdf_baseline:.6f}m tolerance={tolerance_m:.6f}m"
    )
print(f"right.yaml={abs(baseline):.6f}m URDF={urdf_baseline:.6f}m")
PY
    then
      pass "right.yaml and URDF stereo baselines agree within 2 mm"
    else
      fail "right.yaml and URDF stereo baseline cross-check failed"
    fi
  else
    fail "right camera calibration or drone URDF is missing"
  fi
fi

if [[ -r "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  pass "base ROS installation is present"
else
  fail "missing /opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ -n "$REPO_DIR" && -r "${REPO_DIR}/install/setup.bash" ]]; then
  pass "workspace install/setup.bash is present"
else
  fail "workspace has not been built"
fi
if [[ -n "$REPO_DIR" && -r "${REPO_DIR}/install/waar-build-manifest.txt" ]]; then
  pass "build manifest is present"
  if grep -q '^git_state=clean$' "${REPO_DIR}/install/waar-build-manifest.txt"; then
    pass "build manifest records a clean Git checkout"
  else
    fail "build manifest does not record a clean Git checkout"
  fi
else
  warn "build manifest is missing; release provenance is not recorded"
fi

if [[ -n "$REPO_DIR" ]] && command -v git >/dev/null 2>&1 \
    && git -C "$REPO_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  revision="$(git -C "$REPO_DIR" rev-parse --short=12 HEAD)"
  if waar_git_release_is_clean "$REPO_DIR"; then
    pass "Git checkout is clean at $revision"
  else
    fail "Git checkout is dirty at $revision"
  fi
  if [[ -r "${REPO_DIR}/install/waar-build-manifest.txt" ]]; then
    manifest_revision="$(sed -n 's/^git_revision=//p' "${REPO_DIR}/install/waar-build-manifest.txt" | head -n 1)"
    current_revision="$(git -C "$REPO_DIR" rev-parse HEAD)"
    [[ "$manifest_revision" == "$current_revision" ]] \
      && pass "build manifest matches current Git revision" \
      || fail "build manifest revision does not match the checkout"
  fi
fi

if [[ -n "$REPO_DIR" && -r "/opt/ros/${ROS_DISTRO}/setup.bash" && -r "${REPO_DIR}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1090
  source "${REPO_DIR}/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID

  required_packages=(
    drone_hardware_bridge
    slam
    waar_perception
    mas_interfaces
    mas_sync
    mas_task
    mas_mission
  )
  for package in "${required_packages[@]}"; do
    if ros2 pkg prefix "$package" >/dev/null 2>&1; then
      pass "ROS package found: $package"
    else
      fail "ROS package not found: $package"
    fi
  done

  if python3 -c 'import cv2; assert hasattr(cv2, "aruco")' >/dev/null 2>&1; then
    pass "OpenCV AprilTag/aruco module is available"
  else
    fail "Python OpenCV lacks cv2.aruco"
  fi
fi

for command in timeout ip ping v4l2-ctl udevadm; do
  command -v "$command" >/dev/null 2>&1 \
    && pass "diagnostic command available: $command" \
    || warn "diagnostic command missing: $command"
done

if [[ -n "$REPO_DIR" && -d "$REPO_DIR" ]]; then
  available_kb="$(df -Pk "$REPO_DIR" | awk 'NR==2 {print $4}')"
  if waar_is_uint "$available_kb"; then
    if ((available_kb < 1048576)); then
      fail "less than 1 GiB free on the workspace filesystem"
    elif ((available_kb < 3145728)); then
      warn "less than 3 GiB free on the workspace filesystem"
    else
      pass "at least 3 GiB free on the workspace filesystem"
    fi
  fi
fi

if command -v timedatectl >/dev/null 2>&1; then
  if timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qx yes; then
    pass "system clock reports synchronized"
  else
    warn "system clock is not synchronized; correlate logs carefully"
  fi
fi

if [[ "$HARDWARE" == true ]]; then
  printf '\nHardware checks\n'
  if ! waar_is_uint "$IMAGE_WIDTH" || ! waar_is_uint "$IMAGE_HEIGHT" \
      || ! [[ "$FRAMERATE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    fail "IMAGE_WIDTH, IMAGE_HEIGHT, or FRAMERATE is invalid"
  fi
  if [[ "$VIDEO_DEVICE" == *REPLACE_* ]]; then
    fail "VIDEO_DEVICE still contains a placeholder"
  elif [[ ! -c "$VIDEO_DEVICE" ]]; then
    fail "camera is not a character device: $VIDEO_DEVICE"
  else
    [[ -r "$VIDEO_DEVICE" && -w "$VIDEO_DEVICE" ]] \
      && pass "camera device is readable/writable" \
      || fail "camera permissions do not permit read/write access"
    [[ "$VIDEO_DEVICE" == /dev/video[0-9]* ]] \
      && warn "VIDEO_DEVICE uses an unstable /dev/videoN path" \
      || pass "camera uses a stable path"

    if command -v v4l2-ctl >/dev/null 2>&1; then
      if camera_formats="$(timeout 5 v4l2-ctl --device "$VIDEO_DEVICE" --list-formats-ext 2>&1)"; then
        pass "camera responds to V4L2 queries"
        grep -Fq "${IMAGE_WIDTH}x${IMAGE_HEIGHT}" <<<"$camera_formats" \
          && pass "camera advertises ${IMAGE_WIDTH}x${IMAGE_HEIGHT}" \
          || fail "camera does not advertise ${IMAGE_WIDTH}x${IMAGE_HEIGHT}"
        expected_fps="$(awk -v fps="$FRAMERATE" 'BEGIN {printf "%.3f fps", fps}')"
        grep -Fq "$expected_fps" <<<"$camera_formats" \
          && pass "camera advertises ${expected_fps}" \
          || fail "camera does not advertise ${expected_fps} at any mode; inspect mode pairing manually"
      else
        fail "camera V4L2 query failed (busy, disconnected, or wrong node)"
      fi
    fi
  fi

  serial_configured=false
  for serial_name in BRIDGE_SERIAL_PORT SENSOR_SERIAL_PORT; do
    serial_value="${!serial_name}"
    [[ -n "$serial_value" ]] || continue
    serial_configured=true
    if [[ "$serial_value" == *REPLACE_* ]]; then
      fail "$serial_name still contains a placeholder"
    elif [[ -c "$serial_value" && -r "$serial_value" && -w "$serial_value" ]]; then
      pass "$serial_name exists and is accessible"
      [[ "$serial_value" == /dev/ttyUSB[0-9]* || "$serial_value" == /dev/ttyACM[0-9]* ]] \
        && warn "$serial_name uses an unstable kernel-numbered path" \
        || true
    else
      fail "$serial_name is not an accessible character device"
    fi
  done
  [[ "$serial_configured" == true ]] \
    || pass "bridge and legacy sensor serial paths are disabled; flight transport is TCP"

  if waar_is_uint "$ESP32_PORT" && ((ESP32_PORT >= 1 && ESP32_PORT <= 65535)); then
    pass "ESP32 port is valid: $ESP32_PORT"
  else
    fail "ESP32_PORT is invalid"
  fi
  if [[ "$ESP32_HOST" =~ ^[A-Za-z0-9._:-]+$ ]] && ip route get "$ESP32_HOST" >/dev/null 2>&1; then
    pass "a route exists to ESP32 host $ESP32_HOST"
  else
    fail "no usable route to ESP32 host $ESP32_HOST"
  fi
  if command -v ping >/dev/null 2>&1 && ping -c 1 -W 2 "$ESP32_HOST" >/dev/null 2>&1; then
    pass "ESP32 host answers ICMP"
  else
    warn "ESP32 host did not answer ICMP (firmware may still accept TCP)"
  fi

  if [[ "$PROBE_ESP32" == true ]]; then
    service_name="waar-drone@${RUN_USER}.service"
    if command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet "$service_name"; then
      fail "refusing TCP probe while $service_name is active (firmware accepts one client)"
    elif [[ "${AIRFRAME_DISARMED_ACK:-}" != PROPS_REMOVED_AND_DISARMED ]]; then
      fail "TCP probe requires AIRFRAME_DISARMED_ACK=PROPS_REMOVED_AND_DISARMED"
    elif timeout 3 bash -c 'exec 3<>"/dev/tcp/$1/$2"' _ "$ESP32_HOST" "$ESP32_PORT"; then
      pass "ESP32 TCP port accepted a connection"
    else
      fail "ESP32 TCP port did not accept a connection"
    fi
  else
    warn "ESP32 TCP port not opened; use --probe-esp32 only at the props-off, disarmed gate"
  fi
fi

printf '\nSummary: %d failure(s), %d warning(s)\n' "$FAILURES" "$WARNINGS"
if ((FAILURES > 0)); then
  exit 1
fi
exit 0
