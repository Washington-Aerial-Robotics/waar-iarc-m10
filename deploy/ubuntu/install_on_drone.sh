#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

SKIP_APT=false
SKIP_ROSDEP=false
RUN_TESTS=false
INSTALL_SYSTEMD=false

usage() {
  cat <<'EOF'
Usage: install_on_drone.sh [options]

Build the checked-out autonomy workspace for Ubuntu 22.04 / ROS 2 Humble.
It does not fetch Git changes, flash firmware, enable a service, start a
service, or arm the vehicle.

Options:
  --skip-apt          Do not install Ubuntu helper/build packages.
  --skip-rosdep       Do not run rosdep update/install.
  --run-tests         Run colcon test and fail on test failures.
  --install-systemd   Install (but do not enable/start) service support files.
  -h, --help          Show this help.
EOF
}

while (($#)); do
  case "$1" in
    --skip-apt) SKIP_APT=true ;;
    --skip-rosdep) SKIP_ROSDEP=true ;;
    --run-tests) RUN_TESTS=true ;;
    --install-systemd) INSTALL_SYSTEMD=true ;;
    -h|--help) usage; exit 0 ;;
    *) waar_die "unknown argument: $1" ;;
  esac
  shift
done

[[ ${EUID} -ne 0 ]] || waar_die "run this as the target login user, not root; it invokes sudo only where needed"

[[ -r /etc/os-release ]] || waar_die "/etc/os-release is missing"
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
  waar_die "expected Ubuntu 22.04; found ${PRETTY_NAME:-unknown OS}"
[[ -r /opt/ros/humble/setup.bash ]] || \
  waar_die "ROS 2 Humble is not installed under /opt/ros/humble"

git_revision="unavailable"
git_state="not-a-git-checkout"
if command -v git >/dev/null 2>&1 && git -C "$REPO_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git_revision="$(git -C "$REPO_DIR" rev-parse HEAD)"
  if waar_git_release_is_clean "$REPO_DIR"; then
    git_state="clean"
  else
    git_state="DIRTY"
    waar_warn "the build source is dirty; do not approve it for flight without reviewing the diff"
  fi
fi

if [[ "$SKIP_APT" == false ]]; then
  waar_log "installing build and diagnostic dependencies"
  sudo apt-get update
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pytest \
    python3-opencv \
    libopencv-contrib-dev \
    v4l-utils \
    iproute2 \
    iputils-ping \
    iw \
    ethtool \
    netcat-openbsd \
    usbutils \
    jq
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

if [[ "$SKIP_ROSDEP" == false ]]; then
  if [[ ! -r /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    waar_log "initializing rosdep"
    sudo rosdep init
  fi
  rosdep update
  rosdep install \
    --from-paths "${REPO_DIR}/Autonomy" \
    --ignore-src \
    --rosdistro humble \
    -r -y
fi

waar_log "discovering ROS packages"
colcon list --base-paths "${REPO_DIR}/Autonomy"

waar_log "building the workspace"
cd -- "$REPO_DIR"
colcon build \
  --base-paths Autonomy \
  --merge-install \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

if [[ "$RUN_TESTS" == true ]]; then
  waar_log "running ROS package tests"
  colcon test --base-paths Autonomy --event-handlers console_cohesion+
  colcon test-result --verbose
fi

manifest="${REPO_DIR}/install/waar-build-manifest.txt"
{
  printf 'built_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  printf 'git_revision=%s\n' "$git_revision"
  printf 'git_state=%s\n' "$git_state"
  printf 'ros_distro=humble\n'
  printf 'os=%s\n' "${PRETTY_NAME}"
} > "$manifest"
waar_log "wrote build manifest: $manifest"

if [[ "$INSTALL_SYSTEMD" == true ]]; then
  waar_log "installing service support files (the service remains disabled and stopped)"
  sudo install -d -m 0755 /usr/local/libexec/waar-drone
  sudo install -m 0755 \
    "${SCRIPT_DIR}/common.sh" \
    "${SCRIPT_DIR}/run_one_drone.sh" \
    "${SCRIPT_DIR}/preflight.sh" \
    "${SCRIPT_DIR}/verify_stack.sh" \
    "${SCRIPT_DIR}/collect_diagnostics.sh" \
    /usr/local/libexec/waar-drone/
  sudo install -d -m 0755 /usr/local/share/doc/waar-drone
  sudo install -m 0644 \
    "${SCRIPT_DIR}/README.md" \
    "${SCRIPT_DIR}/TEST_MATRIX.md" \
    /usr/local/share/doc/waar-drone/
  sudo install -m 0644 "${SCRIPT_DIR}/waar-drone@.service" /etc/systemd/system/waar-drone@.service
  sudo install -d -m 0755 /etc/waar-drone
  if [[ ! -e "/etc/waar-drone/${USER}.env" && ! -e "/etc/waar-drone/${USER}.env.example" ]]; then
    sudo install -m 0644 "${SCRIPT_DIR}/waar-drone.env.example" "/etc/waar-drone/${USER}.env.example"
  fi
  sudo systemctl daemon-reload
  waar_log "service files installed; edit /etc/waar-drone/${USER}.env before any enable/start action"
fi

waar_log "build complete; no firmware was flashed and no service was started"
