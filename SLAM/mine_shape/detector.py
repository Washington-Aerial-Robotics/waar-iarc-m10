from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from apriltag.calibration import CameraCalibration

from .geometry import shape_center_to_world
from .models import ShapeMineCandidate
from .template import TemplateLoadError, load_template_contour


class ShapeMineDetector:
    """
    Classical PFM-1 detector: Canny contours + cv2.matchShapes against a template silhouette.
    """

    def __init__(
        self,
        calibration: CameraCalibration,
        *,
        template_path: Path | None,
        physical_span_m: float,
        min_shape_confidence: float = 0.35,
        max_match_distance: float = 0.45,
        min_contour_area_px: float = 400.0,
        max_contour_area_px: float = 80000.0,
        canny_low: int = 40,
        canny_high: int = 120,
        ground_z_m: float = 0.0,
        world_drone_transform_provider,
        drone_camera_transform: np.ndarray,
    ):
        self.calibration = calibration
        self.physical_span_m = physical_span_m
        self.min_shape_confidence = min_shape_confidence
        self.max_match_distance = max_match_distance
        self.min_contour_area_px = min_contour_area_px
        self.max_contour_area_px = max_contour_area_px
        self.canny_low = canny_low
        self.canny_high = canny_high
        self.ground_z_m = ground_z_m
        self._world_drone_transform_provider = world_drone_transform_provider
        self.drone_camera_transform = drone_camera_transform

        fx, fy, cx, cy = calibration.camera_params
        self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy

        self._template_contour: np.ndarray | None = None
        self._template_area = 0.0
        self._template_missing_warned = False
        try:
            self._template_contour = load_template_contour(template_path)
            self._template_area = float(cv2.contourArea(self._template_contour))
        except TemplateLoadError as exc:
            self._template_error = str(exc)

    @property
    def template_ready(self) -> bool:
        return self._template_contour is not None

    def _match_confidence(self, match_distance: float) -> float:
        if match_distance >= self.max_match_distance:
            return 0.0
        return max(0.0, min(1.0, 1.0 - match_distance / self.max_match_distance))

    def detect(self, frame_bgr: np.ndarray, timestamp: float | None = None) -> list[ShapeMineCandidate]:
        if timestamp is None:
            timestamp = time.time()

        if not self.template_ready:
            if not self._template_missing_warned:
                print(f"[mine_shape] disabled: {getattr(self, '_template_error', 'no template')}")
                self._template_missing_warned = True
            return []

        undistorted = cv2.undistort(
            frame_bgr,
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
        )
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        world_drone = self._world_drone_transform_provider(timestamp)

        candidates: list[ShapeMineCandidate] = []
        template = self._template_contour
        assert template is not None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area_px or area > self.max_contour_area_px:
                continue

            # Rough scale gate vs template area (±4×)
            if self._template_area > 0:
                ratio = area / self._template_area
                if ratio < 0.15 or ratio > 6.0:
                    continue

            match_dist = cv2.matchShapes(contour, template, cv2.CONTOURS_MATCH_I1, 0.0)
            confidence = self._match_confidence(match_dist)
            if confidence < self.min_shape_confidence:
                continue

            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            x, y, w, h = cv2.boundingRect(contour)
            apparent_span = float(max(w, h))

            world_position = shape_center_to_world(
                (cx, cy),
                apparent_span,
                fx=self._fx,
                fy=self._fy,
                cx=self._cx,
                cy=self._cy,
                physical_span_m=self.physical_span_m,
                world_drone_transform=world_drone,
                drone_camera_transform=self.drone_camera_transform,
                ground_z_m=self.ground_z_m,
            )

            candidates.append(
                ShapeMineCandidate(
                    timestamp=timestamp,
                    center_px=(float(cx), float(cy)),
                    confidence=confidence,
                    world_position=world_position,
                    match_distance=float(match_dist),
                    contour_area_px=float(area),
                    apparent_span_px=apparent_span,
                )
            )

        candidates.sort(key=lambda c: c.confidence, reverse=True)
        return candidates

    def draw_candidates(
        self,
        frame_bgr: np.ndarray,
        candidates: list[ShapeMineCandidate],
    ) -> np.ndarray:
        output = frame_bgr.copy()
        for cand in candidates:
            u, v = int(cand.center_px[0]), int(cand.center_px[1])
            cv2.circle(output, (u, v), 8, (255, 128, 0), 2)
            wp = cand.world_position
            cv2.putText(
                output,
                f"PFM? {cand.confidence:.2f} ({wp[0]:.1f},{wp[1]:.1f})",
                (u + 10, v - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 180, 80),
                1,
            )
        return output
