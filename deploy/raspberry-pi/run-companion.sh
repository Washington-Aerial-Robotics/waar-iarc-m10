#!/bin/bash
set -euo pipefail

case "${WAAR_MODE:-monitor}" in
    monitor|bridge) ;;
    *) echo "WAAR_MODE must be monitor or bridge" >&2; exit 2 ;;
esac

exec /opt/waar/venv/bin/python -u /opt/waar/repo/ESP32/KAF_Drone/tests/vicon/vicon_link.py \
    "${WAAR_MODE:-monitor}" \
    --udp-source "${VICON_RELAY_IP:?Set VICON_RELAY_IP in /etc/waar-single-drone.env}" \
    --drone-ip "${DRONE_IP:?Set DRONE_IP in /etc/waar-single-drone.env}" \
    --drone-id "${DRONE_ID:-U}" \
    --udp-bind "${UDP_BIND:-0.0.0.0}" --udp-port "${UDP_PORT:-5100}" \
    --rate "${SEND_RATE:-100}" --state-rate "${STATE_RATE:-10}" \
    --yaw-offset-deg "${YAW_OFFSET_DEG:-0}" \
    --log "/var/lib/waar/pose-$(date -u +%Y%m%dT%H%M%S)-$$.csv"
