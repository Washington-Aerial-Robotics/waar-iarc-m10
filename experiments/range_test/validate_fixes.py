#!/usr/bin/env python3
"""Validate mine_tag_ids whitelist + improved ShapeMineDetector on range-test photos."""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "SLAM"))

from apriltag.calibration import CameraCalibration
from apriltag.config import PipelineConfig
from apriltag.models import AprilTagDetection
from mine_detection_pipeline import PerceptionPipeline
from mine_shape.detector import ShapeMineDetector

IMG_DIR = Path(__file__).resolve().parent / "images"
TEMPLATE = REPO / "SLAM" / "mine_shape" / "templates" / "pfm1_silhouette.png"


def synth_calib(img: np.ndarray) -> CameraCalibration:
    h, w = img.shape[:2]
    fx = (w / 2.0) / np.tan(np.deg2rad(65.0) / 2.0)
    K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]], float)
    return CameraCalibration(K, np.zeros((1, 5)), (w, h), (fx, fx, w / 2, h / 2))


def fake_detection(tag_id: int) -> AprilTagDetection:
    return AprilTagDetection(
        timestamp=0.0,
        tag_id=tag_id,
        translation_camera=np.array([0.0, 0.0, 1.0]),
        rotation_camera=np.eye(3),
        yaw_deg=0.0,
        pitch_deg=0.0,
        roll_deg=0.0,
        confidence=0.9,
        decision_margin=50.0,
        area_px=1000.0,
        reproj_rmse_px=0.5,
        corners_px=np.zeros((4, 2)),
    )


def test_whitelist() -> None:
    cfg = PipelineConfig(enable_shape_detection=False, enable_csv_log=False, enable_visualization=False)
    assert cfg.mine_tag_ids == [0, 12]
    allowed = set(cfg.mine_tag_ids)
    samples = [0, 12, 1, 5, 36, 99]
    kept = [i for i in samples if i in allowed]
    rejected = [i for i in samples if i not in allowed]
    print("whitelist keep", kept, "reject", rejected)
    assert kept == [0, 12]
    assert 1 in rejected and 99 in rejected

    pipe = object.__new__(PerceptionPipeline)
    pipe.config = cfg
    pipe.pose_provider = type(
        "P",
        (),
        {
            "world_drone_transform": staticmethod(lambda _t: np.eye(4)),
            "get_pose": staticmethod(
                lambda _t: type("Pose", (), {"position": np.array([0.0, 0.0, 1.5])})()
            ),
        },
    )()
    pipe.drone_camera_transform = np.eye(4)
    from apriltag.mine_registry import MineRegistry
    from sparse_voxel_map import SparseVoxelMap
    from mine_shape.registry import ShapeMineRegistry

    pipe.mine_registry = MineRegistry(min_confidence=0.1, use_kalman=False)
    pipe.voxel_map = SparseVoxelMap({"x": 0, "y": 0, "z": 0}, resolution=0.2, field_x=10, field_y=10)
    pipe.csv_logger = None
    pipe.shape_registry = ShapeMineRegistry(min_confidence=0.1, use_kalman=False)

    dets = [fake_detection(1), fake_detection(0), fake_detection(12), fake_detection(7)]
    updated = PerceptionPipeline.process_mine_detections(pipe, dets, 0.0)
    ids = sorted(pipe.mine_registry.mines.keys())
    print("registry after mixed tags:", ids, "updated", len(updated))
    assert ids == [0, 12], f"expected only 0/12, got {ids}"


def run_shape_eval() -> None:
    T = np.eye(4)
    cfg = PipelineConfig()
    rows = []
    for path in sorted(IMG_DIR.glob("*.jpg")):
        img = cv2.imread(str(path))
        if img is None:
            continue
        calib = synth_calib(img)
        det = ShapeMineDetector(
            calib,
            template_path=TEMPLATE,
            physical_span_m=cfg.pfm_physical_span_m,
            min_shape_confidence=cfg.min_shape_confidence,
            max_match_distance=cfg.shape_max_match_distance,
            min_contour_area_px=cfg.shape_min_contour_area_px,
            max_contour_area_px=cfg.shape_max_contour_area_px,
            canny_low=cfg.shape_canny_low,
            canny_high=cfg.shape_canny_high,
            morph_kernel=cfg.shape_morph_kernel,
            min_span_px=cfg.shape_min_span_px,
            max_span_px=cfg.shape_max_span_px,
            min_aspect=cfg.shape_min_aspect,
            max_aspect=cfg.shape_max_aspect,
            min_solidity=cfg.shape_min_solidity,
            min_extent=cfg.shape_min_extent,
            min_silhouette_iou=cfg.shape_min_silhouette_iou,
            use_chromatic_proposal=cfg.shape_use_chromatic_proposal,
            blue_dom_margin=cfg.shape_blue_dom_margin,
            blue_hue_min=cfg.shape_blue_hue_min,
            blue_hue_max=cfg.shape_blue_hue_max,
            blue_min_sat=cfg.shape_blue_min_sat,
            blue_min_value=cfg.shape_blue_min_value,
            world_drone_transform_provider=lambda _t: T,
            drone_camera_transform=T,
        )
        cands = det.detect(img)
        top = cands[0] if cands else None
        rows.append(
            (
                path.name,
                len(cands),
                top.confidence if top else None,
                top.match_distance if top else None,
                top.center_px if top else None,
            )
        )
        print(
            f"{path.name:<44} n={len(cands)} "
            f"conf={top.confidence if top else float('nan'):.3f} "
            f"md={top.match_distance if top else float('nan'):.3f} "
            f"cen={None if not top else (int(top.center_px[0]), int(top.center_px[1]))}"
        )

    def hit(prefix: str) -> list:
        return [r for r in rows if r[0].startswith(prefix)]

    for dist_key, namesub in [
        ("30cm", "30cm"),
        ("50cm", "50cm"),
        ("1m", "_1m"),
        ("1.5m", "1.5m"),
        ("2m", "_2m"),
    ]:
        group = [r for r in rows if namesub in r[0] and r[0].startswith("A")]
        if dist_key == "1m":
            group = [r for r in rows if r[0].startswith("A") and r[0].endswith("_1m.jpg")]
        hits = sum(1 for r in group if r[1] > 0)
        print(f"PartA {dist_key}: {hits}/{len(group)}")

    c3 = hit("C3")
    print(f"C3 false positives: {sum(1 for r in c3 if r[1] > 0)}/{len(c3)} (must be 0)")
    print(f"C1 detections (any): {sum(1 for r in hit('C1') if r[1] > 0)}/{len(hit('C1'))}")
    print(f"C2 detections (any): {sum(1 for r in hit('C2') if r[1] > 0)}/{len(hit('C2'))}")
    b = hit("B")
    print(f"PartB hits: {sum(1 for r in b if r[1] > 0)}/{len(b)}")
    for r in b:
        print(f"  {r[0]}: n={r[1]} conf={r[2]}")


if __name__ == "__main__":
    print("=== Task 1 whitelist ===")
    test_whitelist()
    print("\n=== Task 2 shape on real photos ===")
    run_shape_eval()
