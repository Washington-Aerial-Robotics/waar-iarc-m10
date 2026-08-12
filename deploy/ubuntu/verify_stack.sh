#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

ENV_FILE="${WAAR_ENV_FILE:-}"
WAIT_SECONDS=45
ALLOW_MANUAL=false
FAILURES=0

usage() {
  cat <<'EOF'
Usage: verify_stack.sh [--env-file PATH] [--wait SECONDS] [--allow-manual]

Checks a running one-drone ROS graph without publishing or calling services.
It verifies telemetry, camera, mapping, MAS interfaces, lifecycle service
advertisement, and map->base_link TF. It never prepares, arms, lands, or disarms.
EOF
}

while (($#)); do
  case "$1" in
    --env-file)
      (($# >= 2)) || waar_die "--env-file needs a path"
      ENV_FILE="$2"
      shift 2
      ;;
    --wait)
      (($# >= 2)) || waar_die "--wait needs seconds"
      WAIT_SECONDS="$2"
      shift 2
      ;;
    --allow-manual) ALLOW_MANUAL=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) waar_die "unknown argument: $1" ;;
  esac
done

waar_load_env_file "$ENV_FILE"
: "${REPO_DIR:?Set REPO_DIR}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
DRONE_ID="${DRONE_ID:-d1}"
ENABLE_PERCEPTION="${ENABLE_PERCEPTION:-true}"
MAP_FRAME="${MAP_FRAME:-map}"
BASE_FRAME="${BASE_FRAME:-base_link}"
RUN_USER="${USER:-$(id -un)}"
export ROS_DOMAIN_ID

waar_is_uint "$WAIT_SECONDS" || waar_die "--wait must be an unsigned integer"
((WAIT_SECONDS >= 1 && WAIT_SECONDS <= 300)) || waar_die "--wait must be in 1..300"

if [[ "$ALLOW_MANUAL" == false ]] && command -v systemctl >/dev/null 2>&1; then
  systemctl is-active --quiet "waar-drone@${RUN_USER}.service" \
    || waar_die "waar-drone@${RUN_USER}.service is not active (use --allow-manual for a manual launch)"
fi

waar_source_ros

pass() { printf '[PASS] %s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*" >&2; FAILURES=$((FAILURES + 1)); }

wait_for_topic() {
  local topic="$1"
  local expected_type="$2"
  local deadline=$((SECONDS + WAIT_SECONDS))
  local actual_type=""
  while ((SECONDS < deadline)); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      actual_type="$(ros2 topic type "$topic" 2>/dev/null | head -n 1)"
      if [[ "$actual_type" == "$expected_type" ]]; then
        pass "$topic advertised as $expected_type"
      else
        fail "$topic type is '${actual_type:-unknown}', expected '$expected_type'"
      fi
      return
    fi
    sleep 1
  done
  fail "$topic was not advertised within ${WAIT_SECONDS}s"
}

wait_for_message() {
  local topic="$1"
  if timeout "$WAIT_SECONDS" ros2 topic echo --once "$topic" >/dev/null 2>&1; then
    pass "$topic produced a message"
  else
    fail "$topic produced no message within ${WAIT_SECONDS}s"
  fi
}

check_service() {
  local service="$1"
  local expected_type="$2"
  local line
  line="$(ros2 service list -t 2>/dev/null | grep -F "${service} " | head -n 1)"
  if [[ "$line" == *"[$expected_type]"* ]]; then
    pass "$service advertised as $expected_type"
  else
    fail "$service missing or has the wrong type"
  fi
}

printf 'WAAR running-stack verification (strictly read-only)\n'
printf '  drone=%s domain=%s wait=%ss\n' "$DRONE_ID" "$ROS_DOMAIN_ID" "$WAIT_SECONDS"

wait_for_topic /esp32/odometry nav_msgs/msg/Odometry
wait_for_message /esp32/odometry
wait_for_topic /imu/data sensor_msgs/msg/Imu
wait_for_message /imu/data
wait_for_topic "/${DRONE_ID}/pose" geometry_msgs/msg/PoseStamped
wait_for_message "/${DRONE_ID}/pose"
wait_for_topic /camera/left/image_rect sensor_msgs/msg/Image
wait_for_message /camera/left/image_rect
wait_for_topic /camera/left/camera_info sensor_msgs/msg/CameraInfo
wait_for_message /camera/left/camera_info
wait_for_topic /map nav_msgs/msg/OccupancyGrid
wait_for_message /map
wait_for_topic "/${DRONE_ID}/mission_cmd" std_msgs/msg/String
wait_for_topic "/${DRONE_ID}/mine_candidates" mas_interfaces/msg/MineBelief
wait_for_topic /team/task_result mas_interfaces/msg/TaskResult

check_service "/${DRONE_ID}/prepare" std_srvs/srv/Trigger
check_service "/${DRONE_ID}/arm" std_srvs/srv/Trigger
check_service "/${DRONE_ID}/land" std_srvs/srv/Trigger
check_service "/${DRONE_ID}/disarm" std_srvs/srv/Trigger

if timeout "$WAIT_SECONDS" ros2 run tf2_ros tf2_echo "$MAP_FRAME" "$BASE_FRAME" >/dev/null 2>&1; then
  pass "TF ${MAP_FRAME}->${BASE_FRAME} is available"
else
  fail "TF ${MAP_FRAME}->${BASE_FRAME} was not available"
fi

# The legacy 12-float endpoint must not be promoted to a valid ROS quaternion.
# Accept either an explicit all--1 covariance sentinel (orientation unavailable)
# or a finite, nonzero, normalized quaternion from the current telemetry path.
if imu_yaml="$(timeout "$WAIT_SECONDS" ros2 topic echo --once /imu/data 2>/dev/null)"; then
  if IMU_YAML="$imu_yaml" python3 - <<'PY'
import math
import os
import yaml

msg = yaml.safe_load(os.environ["IMU_YAML"])
cov = msg.get("orientation_covariance", [])
if len(cov) == 9 and float(cov[0]) == -1.0:
    raise SystemExit(0)
q = msg.get("orientation", {})
values = [float(q.get(k, float("nan"))) for k in ("x", "y", "z", "w")]
norm = math.sqrt(sum(v * v for v in values))
raise SystemExit(0 if all(math.isfinite(v) for v in values) and abs(norm - 1.0) <= 1e-3 else 1)
PY
  then
    pass "IMU orientation is explicitly unavailable or a normalized quaternion"
  else
    fail "IMU orientation is neither marked unavailable nor a normalized quaternion"
  fi
else
  fail "could not inspect /imu/data orientation semantics"
fi

printf '\nSummary: %d failure(s)\n' "$FAILURES"
((FAILURES == 0))
