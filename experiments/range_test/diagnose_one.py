#!/usr/bin/env python3
"""Per-gate breakdown for a single photo — shows why candidates pass or fail."""
from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "SLAM"))

from apriltag.calibration import CameraCalibration
from apriltag.config import PipelineConfig
from mine_shape.detector import ShapeMineDetector

IMG = Path(__file__).resolve().parent / "images" / "A3_with_tag_1m.jpg"
TMPL = REPO / "SLAM" / "mine_shape" / "templates" / "pfm1_silhouette.png"


def build_detector(img: np.ndarray) -> ShapeMineDetector:
    h, w = img.shape[:2]
    fx = (w / 2) / np.tan(np.deg2rad(65) / 2)
    K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], float)
    calib = CameraCalibration(K, np.zeros((1, 5)), (w, h), (fx, fx, w / 2, h / 2))
    T = np.eye(4)
    cfg = PipelineConfig()
    return ShapeMineDetector(
        calib,
        template_path=TMPL,
        physical_span_m=cfg.pfm_physical_span_m,
        min_shape_confidence=cfg.min_shape_confidence,
        max_match_distance=cfg.shape_max_match_distance,
        min_contour_area_px=cfg.shape_min_contour_area_px,
        max_contour_area_px=cfg.shape_max_contour_area_px,
        morph_kernel=cfg.shape_morph_kernel,
        min_span_px=cfg.shape_min_span_px,
        max_span_px=cfg.shape_max_span_px,
        min_aspect=cfg.shape_min_aspect,
        max_aspect=cfg.shape_max_aspect,
        min_solidity=cfg.shape_min_solidity,
        min_extent=cfg.shape_min_extent,
        min_silhouette_iou=cfg.shape_min_silhouette_iou,
        use_chromatic_proposal=cfg.shape_use_chromatic_proposal,
        world_drone_transform_provider=lambda _t: T,
        drone_camera_transform=T,
    )


def main() -> None:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else IMG
    img = cv2.imread(str(path))
    print("image", path.name, None if img is None else img.shape)
    if img is None:
        return

    det = build_detector(img)
    print("templates loaded:", len(det._templates))

    undistorted = cv2.undistort(img, det.calibration.camera_matrix, det.calibration.dist_coeffs)
    mask = det._chromatic_mask(undistorted)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = undistorted.shape[:2]
    print(f"proposal foreground px: {int((mask > 0).sum())}  contours: {len(contours)}")

    reasons = Counter()
    passed = []
    for contour in contours:
        if not det._plausible_geometry(contour, h, w):
            reasons["geometry"] += 1
            continue
        match_dist = det._best_match_distance(contour)
        if match_dist > det.max_match_distance:
            reasons["match_distance"] += 1
            continue
        confidence = det._match_confidence(match_dist)
        if confidence < det.min_shape_confidence:
            reasons["confidence"] += 1
            continue
        iou = det._silhouette_iou(mask, contour)
        if iou < det.min_silhouette_iou:
            reasons["silhouette_iou"] += 1
            continue
        x, y, bw, bh = contour_box = cv2.boundingRect(contour)
        reasons["passed"] += 1
        passed.append((confidence, match_dist, iou, max(bw, bh), (x + bw // 2, y + bh // 2)))

    passed.sort(reverse=True)
    print("rejections:", dict(reasons))
    for conf, md, iou, span, center in passed[:5]:
        print(f"  conf={conf:.3f} md={md:.3f} iou={iou:.3f} span={span} center={center}")

    candidates = det.detect(img)
    print(
        "detect() ->",
        len(candidates),
        [(round(c.confidence, 3), round(c.match_distance, 3), int(c.apparent_span_px)) for c in candidates[:5]],
    )


if __name__ == "__main__":
    main()
