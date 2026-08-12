#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

ENV_FILE="${WAAR_ENV_FILE:-}"
OUTPUT_PARENT="${PWD}"

usage() {
  cat <<'EOF'
Usage: collect_diagnostics.sh [--env-file PATH] [--output-parent DIRECTORY]

Collects read-only host, ROS graph, camera, Git, and service diagnostics into a
new timestamped directory. It does not copy the environment file and therefore
does not intentionally collect configuration secrets.
EOF
}

while (($#)); do
  case "$1" in
    --env-file)
      (($# >= 2)) || waar_die "--env-file needs a path"
      ENV_FILE="$2"
      shift 2
      ;;
    --output-parent)
      (($# >= 2)) || waar_die "--output-parent needs a directory"
      OUTPUT_PARENT="$2"
      shift 2
      ;;
    -h|--help) usage; exit 0 ;;
    *) waar_die "unknown argument: $1" ;;
  esac
done

waar_load_env_file "$ENV_FILE"
REPO_DIR="${REPO_DIR:-}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-27}"
DRONE_ID="${DRONE_ID:-d1}"
VIDEO_DEVICE="${VIDEO_DEVICE:-}"
RUN_USER="${USER:-$(id -un)}"
export ROS_DOMAIN_ID

mkdir -p -- "$OUTPUT_PARENT"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
output_dir="${OUTPUT_PARENT%/}/waar-diagnostics-${stamp}"
mkdir -m 0700 -- "$output_dir" || waar_die "cannot create $output_dir"

capture() {
  local filename="$1"
  shift
  "$@" >"${output_dir}/${filename}" 2>&1 || true
}

capture os-release cat /etc/os-release
capture uname.txt uname -a
capture uptime.txt uptime
capture disk.txt df -h
capture memory.txt free -h
capture usb.txt lsusb
capture network-addresses.txt ip -brief address
capture network-routes.txt ip route
capture wifi.txt iw dev
capture recent-kernel.txt dmesg --ctime
capture service.txt systemctl status "waar-drone@${RUN_USER}.service" --no-pager
capture journal.txt journalctl -u "waar-drone@${RUN_USER}.service" --since -30min --no-pager

if [[ -n "$VIDEO_DEVICE" && -c "$VIDEO_DEVICE" ]]; then
  capture camera-all.txt v4l2-ctl --device "$VIDEO_DEVICE" --all
  capture camera-formats.txt v4l2-ctl --device "$VIDEO_DEVICE" --list-formats-ext
  capture camera-udev.txt udevadm info --query all --name "$VIDEO_DEVICE"
fi

if [[ -n "$REPO_DIR" && -d "$REPO_DIR/.git" ]]; then
  capture git-revision.txt git -C "$REPO_DIR" rev-parse HEAD
  capture git-status.txt git -C "$REPO_DIR" status --short
  capture git-diff-stat.txt git -C "$REPO_DIR" diff --stat
  [[ -r "${REPO_DIR}/install/waar-build-manifest.txt" ]] \
    && cp -- "${REPO_DIR}/install/waar-build-manifest.txt" "$output_dir/"
fi

if [[ -n "$REPO_DIR" && -r "/opt/ros/${ROS_DISTRO}/setup.bash" && -r "${REPO_DIR}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1090
  source "${REPO_DIR}/install/setup.bash"
  set -u
  capture ros-nodes.txt timeout 10 ros2 node list
  capture ros-topics.txt timeout 10 ros2 topic list -t
  capture ros-services.txt timeout 10 ros2 service list -t
  capture ros-doctor.txt timeout 30 ros2 doctor --report
  capture tf-map-to-base.txt timeout 10 ros2 run tf2_ros tf2_echo map base_link
fi

{
  printf 'collected_utc=%s\n' "$stamp"
  printf 'drone_id=%s\n' "$DRONE_ID"
  printf 'ros_domain_id=%s\n' "$ROS_DOMAIN_ID"
  printf 'environment_file_copied=false\n'
} >"${output_dir}/collection-metadata.txt"

chmod -R go-rwx "$output_dir"
waar_log "diagnostics saved to $output_dir"
