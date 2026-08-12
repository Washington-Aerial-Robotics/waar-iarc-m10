#!/usr/bin/env bash
set -euo pipefail

TEST_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
DEPLOY_DIR="$(cd -- "${TEST_DIR}/.." && pwd -P)"
REPO_DIR="$(cd -- "${DEPLOY_DIR}/../.." && pwd -P)"

scripts=(
  common.sh
  install_on_drone.sh
  run_one_drone.sh
  preflight.sh
  verify_stack.sh
  collect_diagnostics.sh
  tests/test_deploy.sh
)

for script in "${scripts[@]}"; do
  bash -n "${DEPLOY_DIR}/${script}"
done

base_env=(
  REPO_DIR=/tmp/waar-test-checkout
  ROS_DISTRO=humble
  ROS_DOMAIN_ID=27
  DRONE_ID=d1
  DRY_RUN=true
  AUTO_ARM=false
  VIDEO_DEVICE=/dev/v4l/by-id/test-camera
  APRILTAG_SIZE_M=0.0381
  ARENA_FRAME_SURVEY_ID=test-arena-survey
  CAMERA_EXTRINSICS_ID=test-camera-extrinsics
  STEREO_CALIBRATION_ID=test-stereo-calibration
  YAW_ALIGNMENT_ID=test-yaw-alignment
  BATTERY_SAFETY_ID=test-independent-battery-monitor
  MAX_FLIGHT_TIME_S=60
  CONTROL_CALIBRATION_ID=test-aircraft-control-calibration
  ARENA_MAP_ALIGNED=true
  REQUIRE_BATTERY_VALID=true
)

command_output="$(env "${base_env[@]}" bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command)"
grep -Fq 'ros2 launch drone_hardware_bridge one_drone.launch.py' <<<"$command_output"
grep -Fq 'dry_run:=true' <<<"$command_output"
grep -Fq 'auto_arm:=false' <<<"$command_output"
grep -Fq 'esp_host:=192.168.1.240' <<<"$command_output"
grep -Fq 'esp_port:=70' <<<"$command_output"
grep -Fq 'sender_id:=P' <<<"$command_output"
grep -Fq 'esp_device_id:=U' <<<"$command_output"
grep -Fq 'transport:=tcp' <<<"$command_output"
grep -Fq 'image_width:=2560' <<<"$command_output"
grep -Fq 'image_height:=960' <<<"$command_output"
grep -Fq 'framerate:=10.0' <<<"$command_output"
grep -Fq 'apriltag_size_m:=0.0381' <<<"$command_output"
grep -Fq 'map_frame:=map' <<<"$command_output"
grep -Fq 'odom_frame:=odom' <<<"$command_output"
grep -Fq 'base_frame:=base_link' <<<"$command_output"
grep -Fq 'arena_width:=91.44' <<<"$command_output"
grep -Fq 'arena_height:=24.38' <<<"$command_output"
grep -Fq 'arena_map_aligned:=true' <<<"$command_output"
grep -Fq 'require_battery_valid:=true' <<<"$command_output"
grep -Fq 'max_flight_time_s:=60' <<<"$command_output"

if env "${base_env[@]}" AUTO_ARM=true bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command >/dev/null 2>&1; then
  printf 'AUTO_ARM=true was not rejected\n' >&2
  exit 1
fi

if env "${base_env[@]}" DRY_RUN=false bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command >/dev/null 2>&1; then
  printf 'unacknowledged DRY_RUN=false was not rejected\n' >&2
  exit 1
fi

if env "${base_env[@]}" APRILTAG_SIZE_M=REPLACE_ME bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command >/dev/null 2>&1; then
  printf 'APRILTAG_SIZE_M placeholder was not rejected\n' >&2
  exit 1
fi

if env "${base_env[@]}" SENDER_ID=G bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command >/dev/null 2>&1; then
  printf 'unsafe/stale sender ID was not rejected\n' >&2
  exit 1
fi

if env "${base_env[@]}" ARENA_MAP_ALIGNED=false bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command >/dev/null 2>&1; then
  printf 'unaligned arena map was not rejected\n' >&2
  exit 1
fi

env "${base_env[@]}" DRY_RUN=false HARDWARE_ENABLE_ACK=I_ACCEPT_HARDWARE_OUTPUT \
  bash "${DEPLOY_DIR}/run_one_drone.sh" --print-command >/dev/null

grep -Eq '^DRY_RUN=true$' "${DEPLOY_DIR}/waar-drone.env.example"
grep -Eq '^AUTO_ARM=false$' "${DEPLOY_DIR}/waar-drone.env.example"
! grep -Eq '^AUTO_ARM=true$' "${DEPLOY_DIR}/waar-drone.env.example"
! grep -Eq 'systemctl (enable|start|restart)' "${DEPLOY_DIR}/install_on_drone.sh"
grep -Fq 'RestartPreventExitStatus=64' "${DEPLOY_DIR}/waar-drone@.service"
! grep -Fq 'WAAR_ENV_FILE=' "${DEPLOY_DIR}/waar-drone@.service"

launch_file="${REPO_DIR}/Autonomy/drone_hardware_bridge/launch/one_drone.launch.py"
[[ -r "$launch_file" ]]
expected_launch_args=(
  drone_id num_drones role_coordinator_id dry_run auto_arm
  esp_host esp_port esp_device_id sender_id transport serial_port serial_baud
  camera_device image_width image_height framerate
  sensor_serial_port sensor_serial_baud enable_perception apriltag_size_m
  map_frame odom_frame base_frame arena_width arena_height
  arena_map_aligned require_battery_valid
  max_flight_time_s
)
for launch_arg in "${expected_launch_args[@]}"; do
  grep -Fq "\"${launch_arg}\"" "$launch_file"
done
grep -Fq '"use_legacy_esp32_bridge": "false"' "$launch_file"
grep -Fq 'f"/{self.drone_id}/task_cmd"' \
  "${REPO_DIR}/Autonomy/drone_hardware_bridge/drone_hardware_bridge/node.py"

printf 'deployment script tests passed\n'
