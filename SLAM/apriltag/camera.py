from __future__ import annotations

import sys

import cv2


class CameraSource:
    def __init__(
        self,
        camera_index: int = 0,
        request_width: int = 1920,
        request_height: int = 1080,
        use_v4l2: bool = True,
    ):
        self.camera_index = camera_index
        self.request_width = request_width
        self.request_height = request_height
        self.use_v4l2 = use_v4l2
        self._cap: cv2.VideoCapture | None = None

    def open(self) -> None:
        if self.use_v4l2 and sys.platform.startswith("linux"):
            self._cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        else:
            self._cap = cv2.VideoCapture(self.camera_index)

        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.request_width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.request_height)

    @property
    def actual_resolution(self) -> tuple[int, int]:
        if self._cap is None:
            raise RuntimeError("Camera is not open")
        width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return width, height

    def read(self):
        if self._cap is None:
            raise RuntimeError("Camera is not open")
        return self._cap.read()

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
