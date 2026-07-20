from __future__ import annotations

import os
import sys


def is_headless() -> bool:
    if sys.platform.startswith("linux"):
        return not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY")
    return False
