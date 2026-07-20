from __future__ import annotations

import time

import cv2
import numpy as np
from pupil_apriltags import Detector

from .calibration import CameraCalibration, load_calibration
from .confidence import confidence_score, polygon_area, tilt_angle_degrees
from .geometry import rotmat_to_euler_zyx_degrees
from .models import AprilTagDetection


class AprilTagDetector:
    def __init__(
        self,
        calibration: CameraCalibration,
        tag_family: str = "tag36h11",
        tag_size_m: float = 0.0381,
        nthreads: int = 4,
        quad_decimate: float = 1.0,
        min_confidence: float = 0.0,
    ):
        self.calibration = calibration
        self.tag_size_m = tag_size_m
        self.min_confidence = min_confidence
        self._detector = Detector(
            families=tag_family,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.5,
            debug=False,
        )

    @classmethod
    def from_calib_file(
        cls,
        calib_file,
        tag_family: str = "tag36h11",
        tag_size_m: float = 0.0381,
        min_confidence: float = 0.0,
    ) -> "AprilTagDetector":
        return cls(
            calibration=load_calibration(calib_file),
            tag_family=tag_family,
            tag_size_m=tag_size_m,
            min_confidence=min_confidence,
        )

    def detect(self, frame_bgr: np.ndarray, timestamp: float | None = None) -> list[AprilTagDetection]:
        if timestamp is None:
            timestamp = time.time()

        undistorted = cv2.undistort(
            frame_bgr,
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
        )
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        raw_detections = self._detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.calibration.camera_params,
            tag_size=self.tag_size_m,
        )

        detections: list[AprilTagDetection] = []
        for det in raw_detections:
            rotation = det.pose_R
            translation = det.pose_t.reshape(3)
            yaw_deg, pitch_deg, roll_deg = rotmat_to_euler_zyx_degrees(rotation)
            area_px = polygon_area(det.corners)
            decision_margin = getattr(det, "decision_margin", None)
            confidence = confidence_score(decision_margin, area_px, tilt_angle_degrees(rotation))

            if confidence < self.min_confidence:
                continue

            detections.append(
                AprilTagDetection(
                    timestamp=timestamp,
                    tag_id=int(det.tag_id),
                    translation_camera=translation.astype(np.float64),
                    rotation_camera=rotation.astype(np.float64),
                    yaw_deg=yaw_deg,
                    pitch_deg=pitch_deg,
                    roll_deg=roll_deg,
                    confidence=confidence,
                    decision_margin=None if decision_margin is None else float(decision_margin),
                    area_px=float(area_px),
                    corners_px=det.corners.astype(np.float64),
                )
            )

        return detections

    def draw_detections(self, frame_bgr: np.ndarray, detections: list[AprilTagDetection]) -> np.ndarray:
        output = cv2.undistort(
            frame_bgr,
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
        )

        for det in detections:
            corners = det.corners_px.astype(int)
            cv2.polylines(output, [corners], True, (0, 255, 0), 2)
            center = tuple(np.mean(corners, axis=0).astype(int))
            cv2.putText(
                output,
                f"ID {det.tag_id} z={det.translation_camera[2]:.2f} conf={det.confidence:.2f}",
                (center[0] + 10, center[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

        return output
