#!/usr/bin/env bash

# Shared helpers for the Ubuntu deployment scripts. This file is sourced.

waar_log() {
  printf '[waar] %s\n' "$*"
}

waar_warn() {
  printf '[waar] WARNING: %s\n' "$*" >&2
}

waar_die() {
  printf '[waar] ERROR: %s\n' "$*" >&2
  exit 64
}

waar_is_bool() {
  case "${1:-}" in
    true|false) return 0 ;;
    *) return 1 ;;
  esac
}

waar_is_uint() {
  [[ "${1:-}" =~ ^[0-9]+$ ]]
}

waar_load_env_file() {
  local env_file="${1:-}"
  [[ -n "$env_file" ]] || return 0
  [[ -r "$env_file" ]] || waar_die "environment file is not readable: $env_file"

  # The environment file is an operator-controlled configuration file. Keep it
  # root-owned, group-readable, and mode 0640 when installed under /etc.
  set -a
  # shellcheck disable=SC1090
  source "$env_file"
  set +a
}

waar_source_ros() {
  local distro="${ROS_DISTRO:-humble}"
  local repo_dir="${REPO_DIR:?REPO_DIR must be set}"
  local ros_setup="/opt/ros/${distro}/setup.bash"
  local workspace_setup="${repo_dir}/install/setup.bash"

  [[ -r "$ros_setup" ]] || waar_die "ROS setup not found: $ros_setup"
  [[ -r "$workspace_setup" ]] || waar_die "workspace is not built: $workspace_setup"

  # ROS-generated setup scripts are not guaranteed to tolerate nounset.
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup"
  # shellcheck disable=SC1090
  source "$workspace_setup"
  set -u
}

# Return success when source-controlled content is clean while ignoring only
# known generated Python/colcon artifacts which are absent from this repo's
# historical .gitignore. A modified tracked file is never ignored.
waar_git_release_is_clean() {
  local repo_dir="$1"
  local untracked=""

  git -C "$repo_dir" diff --quiet -- || return 1
  git -C "$repo_dir" diff --cached --quiet -- || return 1
  untracked="$(
    git -C "$repo_dir" ls-files --others --exclude-standard \
      | grep -Ev '^(build|install|log)/|(^|/)(__pycache__|\.pytest_cache)/|\.py[co]$' \
      || true
  )"
  [[ -z "$untracked" ]]
}
