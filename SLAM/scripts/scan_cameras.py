"""Scan V4L2 camera indices and report which ones produce frames."""

from __future__ import annotations

import argparse
import sys

import cv2


def try_camera(index: int, width: int, height: int) -> tuple[bool, tuple[int, int] | None]:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        return False, None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    ok, frame = cap.read()
    shape = None if frame is None else (frame.shape[1], frame.shape[0])
    cap.release()
    return ok and shape is not None, shape


def main() -> None:
    parser = argparse.ArgumentParser(description="Scan USB camera indices on Linux/Pi")
    parser.add_argument("--max-index", type=int, default=6)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    args = parser.parse_args()

    if not sys.platform.startswith("linux"):
        print("Note: V4L2 scan is intended for Linux/Pi. Trying anyway.")

    print(f"Scanning indices 0-{args.max_index - 1} at {args.width}x{args.height}...")
    found = False
    for idx in range(args.max_index):
        ok, shape = try_camera(idx, args.width, args.height)
        if ok:
            found = True
            print(f"  index {idx}: OK  shape={shape[0]}x{shape[1]}")
        else:
            print(f"  index {idx}: no frame")

    if not found:
        print("\nNo working camera found. Try 1280x720:")
        for idx in range(args.max_index):
            ok, shape = try_camera(idx, 1280, 720)
            if ok:
                print(f"  index {idx}: OK at 1280x720  shape={shape[0]}x{shape[1]}")


if __name__ == "__main__":
    main()
