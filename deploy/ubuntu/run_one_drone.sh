#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

ENV_FILE="${WAAR_ENV_FILE:-}"
PRINT_COMMAND=false

usage() {
  cat <<'EOF'
Usage: run_one_drone.sh [--env-file PATH] [--print-command]

Launches the one-drone ROS stack. --print-command validates configuration and
prints the command without sourcing ROS or starting any process.
EOF
}

while (($#)); do
  case "$1" in
    --env-file)
      (($# >= 2)) || waar_die "--env-file needs a path"
      ENV_FILE="$2"
      shift 2
      ;;
    --print-command)
      PRINT_COMMAND=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      waar_die "unknown argument: $1"
      ;;
  esac
done

waar_load_env_file "$ENV_FILE"

: "${REPO_DIR:?Set REPO_DIR to the repository checkout}"
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
BRIDGE_SERIAL_PORT="${BRIDGE_SERIAL_PORT:-}"
BRIDGE_SERIAL_BAUD="${BRIDGE_SERIAL_BAUD:-115200}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/v4l/by-id/REPLACE_WITH_STEREO_CAMERA}"
IMAGE_WIDTH="${IMAGE_WIDTH:-2560}"
IMAGE_HEIGHT="${IMAGE_HEIGHT:-960}"
FRAMERATE="${FRAMERATE:-10.0}"
ENABLE_PERCEPTION="${ENABLE_PERCEPTION:-true}"
APRILTAG_SIZE_M="${APRILTAG_SIZE_M:-REPLACE_WITH_MEASURED_BLACK_SQUARE_METERS}"
SENSOR_SERIAL_PORT="${SENSOR_SERIAL_PORT:-}"
SENSOR_SERIAL_BAUD="${SENSOR_SERIAL_BAUD:-115200}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"
BASE_FRAME="${BASE_FRAME:-base_link}"
ARENA_WIDTH_M="${ARENA_WIDTH_M:-91.44}"
ARENA_HEIGHT_M="${ARENA_HEIGHT_M:-24.38}"
ARENA_MAP_ALIGNED="${ARENA_MAP_ALIGNED:-false}"
ARENA_FRAME_SURVEY_ID="${ARENA_FRAME_SURVEY_ID:-REPLACE_WITH_ARENA_FRAME_SURVEY_RECORD}"
CAMERA_EXTRINSICS_ID="${CAMERA_EXTRINSICS_ID:-REPLACE_WITH_CAMERA_EXTRINSICS_RECORD}"
STEREO_CALIBRATION_ID="${STEREO_CALIBRATION_ID:-REPLACE_WITH_STEREO_CALIBRATION_RECORD}"
YAW_ALIGNMENT_ID="${YAW_ALIGNMENT_ID:-REPLACE_WITH_YAW_ALIGNMENT_RECORD}"
BATTERY_SAFETY_ID="${BATTERY_SAFETY_ID:-REPLACE_WITH_INDEPENDENT_BATTERY_MONITOR_RECORD}"
MAX_FLIGHT_TIME_S="${MAX_FLIGHT_TIME_S:-REPLACE_WITH_CONSERVATIVE_TESTED_LIMIT}"
CONTROL_CALIBRATION_ID="${CONTROL_CALIBRATION_ID:-REPLACE_WITH_AIRCRAFT_CONTROL_CALIBRATION_RECORD}"
REQUIRE_BATTERY_VALID="${REQUIRE_BATTERY_VALID:-true}"

[[ "$ROS_DISTRO" == humble ]] || waar_die "this deployment targets ROS_DISTRO=humble"
waar_is_uint "$ROS_DOMAIN_ID" || waar_die "ROS_DOMAIN_ID must be an unsigned integer"
((ROS_DOMAIN_ID <= 232)) || waar_die "ROS_DOMAIN_ID must be in 0..232"
[[ "$DRONE_ID" =~ ^[a-z][a-z0-9_]*$ ]] || waar_die "invalid DRONE_ID: $DRONE_ID"
for frame_name in MAP_FRAME ODOM_FRAME BASE_FRAME; do
  frame_value="${!frame_name}"
  [[ "$frame_value" =~ ^[A-Za-z][A-Za-z0-9_/]*$ ]] \
    || waar_die "$frame_name is not a valid non-empty frame ID"
  [[ "$frame_value" != /* ]] || waar_die "$frame_name must not begin with '/'"
done
[[ "$MAP_FRAME" != "$ODOM_FRAME" && "$MAP_FRAME" != "$BASE_FRAME" && "$ODOM_FRAME" != "$BASE_FRAME" ]] \
  || waar_die "MAP_FRAME, ODOM_FRAME, and BASE_FRAME must be distinct"
for dimension_name in ARENA_WIDTH_M ARENA_HEIGHT_M; do
  dimension_value="${!dimension_name}"
  [[ "$dimension_value" =~ ^[0-9]+([.][0-9]+)?$ ]] \
    && awk -v value="$dimension_value" 'BEGIN {exit !(value > 0.0)}' \
    || waar_die "$dimension_name must be a positive decimal value in metres"
done
waar_is_bool "$DRY_RUN" || waar_die "DRY_RUN must be exactly true or false"
waar_is_bool "$AUTO_ARM" || waar_die "AUTO_ARM must be exactly true or false"
waar_is_bool "$ENABLE_PERCEPTION" || waar_die "ENABLE_PERCEPTION must be exactly true or false"
waar_is_bool "$ARENA_MAP_ALIGNED" || waar_die "ARENA_MAP_ALIGNED must be exactly true or false"
waar_is_bool "$REQUIRE_BATTERY_VALID" || waar_die "REQUIRE_BATTERY_VALID must be exactly true or false"
waar_is_uint "$ESP32_PORT" || waar_die "ESP32_PORT must be an unsigned integer"
((ESP32_PORT >= 1 && ESP32_PORT <= 65535)) || waar_die "ESP32_PORT is out of range"
[[ "$ESP32_HOST" =~ ^[A-Za-z0-9._:-]+$ ]] || waar_die "ESP32_HOST contains unsupported characters"
[[ "$TRANSPORT" == tcp ]] || waar_die "the flight deployment requires TRANSPORT=tcp"
[[ "$SENDER_ID" == P ]] || waar_die "SENDER_ID must be P for the current firmware"
[[ "$ESP32_DEVICE_ID" == U ]] || waar_die "ESP32_DEVICE_ID must be U for the current firmware"
waar_is_uint "$BRIDGE_SERIAL_BAUD" || waar_die "BRIDGE_SERIAL_BAUD must be an unsigned integer"
waar_is_uint "$SENSOR_SERIAL_BAUD" || waar_die "SENSOR_SERIAL_BAUD must be an unsigned integer"
[[ "$VIDEO_DEVICE" != *REPLACE_* ]] || waar_die "replace the VIDEO_DEVICE placeholder"
[[ "$APRILTAG_SIZE_M" != *REPLACE_* ]] || waar_die "replace APRILTAG_SIZE_M with the measured black-square size"
[[ "$APRILTAG_SIZE_M" =~ ^0[.][0-9]+$ ]] || waar_die "APRILTAG_SIZE_M must be a positive decimal value in metres"
awk -v size="$APRILTAG_SIZE_M" 'BEGIN {exit !(size > 0.0 && size < 1.0)}' \
  || waar_die "APRILTAG_SIZE_M must be in the plausible range (0, 1) metre"
for record_name in \
    ARENA_FRAME_SURVEY_ID \
    CAMERA_EXTRINSICS_ID \
    STEREO_CALIBRATION_ID \
    YAW_ALIGNMENT_ID \
    BATTERY_SAFETY_ID \
    CONTROL_CALIBRATION_ID; do
  record_value="${!record_name}"
  [[ -n "$record_value" && "$record_value" != *REPLACE_* ]] \
    || waar_die "$record_name must reference the approved survey/calibration record"
done
waar_is_uint "$MAX_FLIGHT_TIME_S" || waar_die "MAX_FLIGHT_TIME_S must be an explicitly tested unsigned integer"
((MAX_FLIGHT_TIME_S >= 1)) || waar_die "MAX_FLIGHT_TIME_S must be positive"
[[ "$ARENA_MAP_ALIGNED" == true ]] \
  || waar_die "ARENA_MAP_ALIGNED must be true after completing the signed arena survey"

# This service is deliberately never an automatic arming mechanism. Arming is
# an explicit lifecycle service call made by a human after the preflight gates.
[[ "$AUTO_ARM" == false ]] || waar_die "AUTO_ARM=true is prohibited by the service wrapper"
if [[ "$DRY_RUN" == false && "$HARDWARE_ENABLE_ACK" != I_ACCEPT_HARDWARE_OUTPUT ]]; then
  waar_die "DRY_RUN=false requires HARDWARE_ENABLE_ACK=I_ACCEPT_HARDWARE_OUTPUT"
fi

export ROS_DOMAIN_ID

launch_command=(
  ros2 launch drone_hardware_bridge one_drone.launch.py
  "drone_id:=${DRONE_ID}"
  "num_drones:=1"
  "role_coordinator_id:=${DRONE_ID}"
  "dry_run:=${DRY_RUN}"
  "auto_arm:=${AUTO_ARM}"
  "esp_host:=${ESP32_HOST}"
  "esp_port:=${ESP32_PORT}"
  "esp_device_id:=${ESP32_DEVICE_ID}"
  "sender_id:=${SENDER_ID}"
  "transport:=${TRANSPORT}"
  "serial_port:=${BRIDGE_SERIAL_PORT}"
  "serial_baud:=${BRIDGE_SERIAL_BAUD}"
  "camera_device:=${VIDEO_DEVICE}"
  "image_width:=${IMAGE_WIDTH}"
  "image_height:=${IMAGE_HEIGHT}"
  "framerate:=${FRAMERATE}"
  "sensor_serial_port:=${SENSOR_SERIAL_PORT}"
  "sensor_serial_baud:=${SENSOR_SERIAL_BAUD}"
  "enable_perception:=${ENABLE_PERCEPTION}"
  "apriltag_size_m:=${APRILTAG_SIZE_M}"
  "map_frame:=${MAP_FRAME}"
  "odom_frame:=${ODOM_FRAME}"
  "base_frame:=${BASE_FRAME}"
  "arena_width:=${ARENA_WIDTH_M}"
  "arena_height:=${ARENA_HEIGHT_M}"
  "arena_map_aligned:=${ARENA_MAP_ALIGNED}"
  "require_battery_valid:=${REQUIRE_BATTERY_VALID}"
  "max_flight_time_s:=${MAX_FLIGHT_TIME_S}"
)

if [[ "$PRINT_COMMAND" == true ]]; then
  printf '%q ' "${launch_command[@]}"
  printf '\n'
  exit 0
fi

waar_source_ros
cd -- "$REPO_DIR"
waar_log "launching ${DRONE_ID}; dry_run=${DRY_RUN}; auto_arm=${AUTO_ARM}; ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
exec "${launch_command[@]}"
