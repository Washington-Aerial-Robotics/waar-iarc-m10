from __future__ import annotations

import numpy as np


def eye_width_for_frame(width: int, height: int) -> int | None:
    if width == 2560 and height == 720:
        return 1280
    if width == 1280 and height == 720:
        return 640
    if width % 2 == 0 and width >= height * 2:
        return width // 2
    return None


def split_stereo_frame(frame_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, int] | None:
    """Side-by-side stereo: left eye, right eye, eye width in pixels."""
    height, width = frame_bgr.shape[:2]
    eye_w = eye_width_for_frame(width, height)
    if eye_w is None:
        return None
    left = frame_bgr[:, :eye_w]
    right = frame_bgr[:, eye_w : eye_w * 2]
    return left, right, eye_w
