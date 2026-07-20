from __future__ import annotations

import csv
from pathlib import Path

from .models import AprilTagDetection, FusedMine


class DetectionCsvLogger:
    def __init__(self, log_file: Path):
        self.log_file = log_file
        self._file = open(log_file, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "timestamp",
            "tag_id",
            "camera_x",
            "camera_y",
            "camera_z",
            "world_x",
            "world_y",
            "world_z",
            "yaw",
            "pitch",
            "roll",
            "confidence",
        ])
        self._rows_since_flush = 0

    def log(
        self,
        detection: AprilTagDetection,
        world_position,
        fused: FusedMine | None = None,
    ) -> None:
        world_x = world_y = world_z = ""
        if world_position is not None:
            world_x = float(world_position[0])
            world_y = float(world_position[1])
            world_z = float(world_position[2])

        self._writer.writerow([
            detection.timestamp,
            detection.tag_id,
            float(detection.translation_camera[0]),
            float(detection.translation_camera[1]),
            float(detection.translation_camera[2]),
            world_x,
            world_y,
            world_z,
            detection.yaw_deg,
            detection.pitch_deg,
            detection.roll_deg,
            detection.confidence if fused is None else fused.confidence,
        ])
        self._rows_since_flush += 1
        if self._rows_since_flush >= 20:
            self._file.flush()
            self._rows_since_flush = 0

    def close(self) -> None:
        self._file.flush()
        self._file.close()
