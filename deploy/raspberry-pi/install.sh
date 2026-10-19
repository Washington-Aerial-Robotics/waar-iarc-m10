#!/bin/bash
# Run on the Pi: sudo bash deploy/raspberry-pi/install.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo on the Raspberry Pi." >&2
    exit 1
fi
if [[ $(dpkg --print-architecture) != arm64 ]]; then
    echo "Use a 64-bit Raspberry Pi OS installation (arm64)." >&2
    exit 1
fi

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ "$repo" == /opt/waar/repo ]]; then
    echo "Run from your checkout, not the installed /opt/waar/repo copy." >&2
    exit 1
fi

apt-get update
apt-get install -y python3 python3-venv python3-numpy python3-pytest rsync chrony
if ! id waar >/dev/null 2>&1; then
    useradd --system --user-group --home-dir /var/lib/waar --shell /usr/sbin/nologin waar
fi

# Stop an existing instance before updating code. Installation never starts it.
if systemctl cat waar-companion.service >/dev/null 2>&1; then
    systemctl stop waar-companion.service
fi
install -d -m 755 /opt/waar/repo
for path in Autonomy ESP32 scripts deploy; do
    rsync -a --exclude='.venv' --exclude='__pycache__' --exclude='.pytest_cache' \
        --exclude='wifi_credentials.h' --exclude='*.env' \
        "$repo/$path/" "/opt/waar/repo/$path/"
done
install -m 644 "$repo/requirements-test.txt" /opt/waar/repo/
python3 -m venv --system-site-packages /opt/waar/venv
chmod -R a+rX /opt/waar/repo
install -d -o waar -g waar -m 750 /var/lib/waar
if [[ ! -f /etc/waar-single-drone.env ]]; then
    install -o root -g root -m 600 "$repo/deploy/raspberry-pi/companion.env.example" /etc/waar-single-drone.env
fi
install -m 644 "$repo/deploy/raspberry-pi/waar-companion.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now chrony
/opt/waar/venv/bin/python /opt/waar/repo/scripts/check_software.py
echo "Installed. Edit /etc/waar-single-drone.env, then start waar-companion when ready."
echo "Companion service left stopped; no flight command was sent."
