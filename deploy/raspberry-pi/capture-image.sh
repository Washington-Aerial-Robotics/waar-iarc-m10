#!/bin/bash
# Run on a Linux imaging machine, with the Pi shut down and its SD card unmounted.
# Reads SOURCE; creates a new image file. Never writes to SOURCE.
set -euo pipefail
if [[ $# -ne 2 || $EUID -ne 0 ]]; then
    echo "Usage: sudo bash capture-image.sh /dev/sdX /absolute/path/waar-single.img" >&2
    exit 2
fi
source_disk=$(readlink -f -- "$1")
output_image=$(realpath -m -- "$2")
if [[ "$source_disk" != /dev/* || ! -b "$source_disk" || $(lsblk -dnro TYPE "$source_disk") != disk ]]; then
    echo "SOURCE must be a whole block disk, not a partition." >&2
    exit 2
fi
if [[ -n $(lsblk -nrpo MOUNTPOINTS "$source_disk" | tr -d '[:space:]') ]]; then
    echo "Unmount all source partitions first; do not capture a running Pi/root disk." >&2
    exit 2
fi
if [[ "$output_image" == /dev/* || -e "$output_image" || -L "$output_image" ]]; then
    echo "OUTPUT must be a new regular image file outside /dev." >&2
    exit 2
fi
set -o noclobber
: > "$output_image"
dd if="$source_disk" of="$output_image" bs=4M status=progress conv=fsync
sha256sum "$output_image" > "$output_image.sha256"
echo "Created $output_image and checksum. Target cards must be at least as large as the source."
