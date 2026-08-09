from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

from apriltag.calibration import CameraCalibration

from .geometry import shape_center_to_world
from .models import ShapeMineCandidate
from .template import DEFAULT_TEMPLATE_PATH, TemplateLoadError, load_template_contour


class ShapeMineDetector:
    """
    Classical PFM-1 detector for real imagery.

    Gray-only Canny + matchShapes cannot recover filled mine silhouettes on
    cluttered phone photos (fragmented edges; aggressive morphology floods the
    frame). Candidate **proposal** uses chromatic blue (B-dominant + narrow
    hue/sat) to get filled blobs; **acceptance / ranking** uses matchShapes plus
    silhouette IoU — color alone never registers a mine (that false-fired on C3).

    Strong camera tilt still degrades Hu+IoU; that is a classical-method limit.
    """

    def __init__(
        self,
        calibration: CameraCalibration,
        *,
        template_path: Path | None,
        physical_span_m: float,
        min_shape_confidence: float = 0.55,
        max_match_distance: float = 0.18,
        min_contour_area_px: float = 700.0,
        max_contour_area_px: float = 200000.0,
        canny_low: int = 30,
        canny_high: int = 100,
        morph_kernel: int = 5,
        min_span_px: float = 60.0,
        max_span_px: float = 700.0,
        min_aspect: float = 1.35,
        max_aspect: float = 3.5,
        min_solidity: float = 0.55,
        min_extent: float = 0.42,
        min_silhouette_iou: float = 0.48,
        use_chromatic_proposal: bool = True,
        blue_dom_margin: int = 25,
        blue_hue_min: int = 95,
        blue_hue_max: int = 135,
        blue_min_sat: int = 40,
        blue_min_value: int = 40,
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
        self.morph_kernel = max(3, int(morph_kernel) | 1)
        self.min_span_px = min_span_px
        self.max_span_px = max_span_px
        self.min_aspect = min_aspect
        self.max_aspect = max_aspect
        self.min_solidity = min_solidity
        self.min_extent = min_extent
        self.min_silhouette_iou = min_silhouette_iou
        self.use_chromatic_proposal = use_chromatic_proposal
        self.blue_dom_margin = blue_dom_margin
        self.blue_hue_min = blue_hue_min
        self.blue_hue_max = blue_hue_max
        self.blue_min_sat = blue_min_sat
        self.blue_min_value = blue_min_value
        self.ground_z_m = ground_z_m
        self._world_drone_transform_provider = world_drone_transform_provider
        self.drone_camera_transform = drone_camera_transform

        fx, fy, cx, cy = calibration.camera_params
        self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy

        self._templates: list[np.ndarray] = []
        self._silhouette_bin: np.ndarray | None = None
        self._template_missing_warned = False
        self._template_error = "no template"
        try:
            primary = template_path or DEFAULT_TEMPLATE_PATH
            self._templates.append(load_template_contour(primary))
            sil = cv2.imread(str(primary), cv2.IMREAD_GRAYSCALE)
            if sil is not None:
                _, self._silhouette_bin = cv2.threshold(sil, 127, 255, cv2.THRESH_BINARY)
                if self._silhouette_bin[0, 0] > 0 and self._silhouette_bin[0, -1] > 0:
                    self._silhouette_bin = 255 - self._silhouette_bin
            for suffix in ("_30deg", "_45deg"):
                alt = primary.with_name(primary.stem + suffix + primary.suffix)
                if alt.exists():
                    try:
                        self._templates.append(load_template_contour(alt))
                    except TemplateLoadError:
                        pass
        except TemplateLoadError as exc:
            self._template_error = str(exc)

    @property
    def template_ready(self) -> bool:
        return bool(self._templates)

    def _match_confidence(self, match_distance: float) -> float:
        scale = 0.45
        if match_distance >= scale:
            return 0.0
        return max(0.0, min(1.0, 1.0 - match_distance / scale))

    def _chromatic_mask(self, bgr: np.ndarray) -> np.ndarray:
        b, g, r = cv2.split(bgr)
        margin = int(self.blue_dom_margin)
        dom = (b.astype(np.int16) - g.astype(np.int16) > margin) & (
            b.astype(np.int16) - r.astype(np.int16) > margin
        )
        bright = b > int(self.blue_min_value)
        mask = np.where(dom & bright, 255, 0).astype(np.uint8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hue_ok = (hsv[:, :, 0] >= int(self.blue_hue_min)) & (hsv[:, :, 0] <= int(self.blue_hue_max))
        sat_ok = hsv[:, :, 1] > int(self.blue_min_sat)
        mask = np.where((mask > 0) & hue_ok & sat_ok, 255, 0).astype(np.uint8)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        return mask

    def _best_match_distance(self, contour: np.ndarray) -> float:
        hull = cv2.convexHull(contour)
        best = float("inf")
        for tmpl in self._templates:
            best = min(best, float(cv2.matchShapes(contour, tmpl, cv2.CONTOURS_MATCH_I1, 0.0)))
            best = min(best, float(cv2.matchShapes(hull, tmpl, cv2.CONTOURS_MATCH_I1, 0.0)))
        return best

    def _silhouette_iou(self, mask: np.ndarray, contour: np.ndarray) -> float:
        if self._silhouette_bin is None:
            return 1.0
        x, y, w, h = cv2.boundingRect(contour)
        patch = mask[y : y + h, x : x + w]
        if patch.size == 0:
            return 0.0
        sil = self._silhouette_bin
        best = 0.0
        for tw, th in ((w, h), (h, w)):
            if tw < 8 or th < 8:
                continue
            for src in (sil, cv2.flip(sil, 1)):
                resized = cv2.resize(src, (tw, th), interpolation=cv2.INTER_AREA)
                if resized.shape != patch.shape:
                    continue
                inter = np.logical_and(patch > 0, resized > 0).sum()
                union = np.logical_or(patch > 0, resized > 0).sum()
                if union > 0:
                    best = max(best, float(inter) / float(union))
        return best

    def _plausible_geometry(self, contour: np.ndarray, frame_h: int, frame_w: int) -> bool:
        area = float(cv2.contourArea(contour))
        if area < self.min_contour_area_px or area > self.max_contour_area_px:
            return False

        _x, _y, w, h = cv2.boundingRect(contour)
        span = float(max(w, h))
        max_span = min(self.max_span_px, 0.55 * max(frame_h, frame_w))
        if span < self.min_span_px or span > max_span:
            return False

        (_cx, _cy), (rw, rh), _angle = cv2.minAreaRect(contour)
        if rw < 1.0 or rh < 1.0:
            return False
        aspect = max(rw, rh) / float(max(min(rw, rh), 1e-3))
        if aspect < self.min_aspect or aspect > self.max_aspect:
            return False

        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        if hull_area < 1.0:
            return False
        solidity = area / hull_area
        if solidity < self.min_solidity:
            return False

        extent = area / float(max(w * h, 1))
        if extent < self.min_extent:
            return False

        if aspect < 1.25 and solidity > 0.85:
            return False

        return True

    def detect(self, frame_bgr: np.ndarray, timestamp: float | None = None) -> list[ShapeMineCandidate]:
        if timestamp is None:
            timestamp = time.time()

        if not self.template_ready:
            if not self._template_missing_warned:
                print(f"[mine_shape] disabled: {self._template_error}")
                self._template_missing_warned = True
            return []

        undistorted = cv2.undistort(
            frame_bgr,
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
        )
        fh, fw = undistorted.shape[:2]

        if self.use_chromatic_proposal:
            mask = self._chromatic_mask(undistorted)
        else:
            gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.morph_kernel, self.morph_kernel))
            mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, k, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        world_drone = self._world_drone_transform_provider(timestamp)

        scored: list[tuple[float, ShapeMineCandidate]] = []
        for contour in contours:
            if not self._plausible_geometry(contour, fh, fw):
                continue

            match_dist = self._best_match_distance(contour)
            if match_dist > self.max_match_distance:
                continue

            confidence = self._match_confidence(match_dist)
            if confidence < self.min_shape_confidence:
                continue

            iou = self._silhouette_iou(mask, contour)
            if iou < self.min_silhouette_iou:
                continue

            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            _x, _y, w, h = cv2.boundingRect(contour)
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

            cand = ShapeMineCandidate(
                timestamp=timestamp,
                center_px=(float(cx), float(cy)),
                confidence=confidence,
                world_position=world_position,
                match_distance=float(match_dist),
                contour_area_px=float(cv2.contourArea(contour)),
                apparent_span_px=apparent_span,
            )
            span_prior = 0.5 + 0.5 * min(1.0, apparent_span / 120.0)
            score = (confidence ** 2) * iou * span_prior
            scored.append((score, cand))

        scored.sort(key=lambda t: t[0], reverse=True)
        kept: list[ShapeMineCandidate] = []
        for _score, cand in scored:
            if any(
                abs(cand.center_px[0] - k.center_px[0]) < 50
                and abs(cand.center_px[1] - k.center_px[1]) < 50
                for k in kept
            ):
                continue
            kept.append(cand)
        return kept

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
