#!/usr/bin/env python3
"""
Range / false-positive measurement harness for PFM-1 photos.

Measurement only — does not modify SLAM detectors or original Downloads photos.
Reports (1) stock ShapeMineDetector + AprilTag tag36h11 results, and
(2) harness-only diagnostics that relax known gates / add a blue HSV prior
so we can still extract ranging/FP numbers when the stock path is gated out.
"""
from __future__ import annotations

import json
import re
import shutil
import sys
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
SLAM = REPO / "SLAM"
sys.path.insert(0, str(SLAM))

from apriltag.calibration import CameraCalibration  # noqa: E402
from apriltag.detector import AprilTagDetector  # noqa: E402
from mine_shape.detector import ShapeMineDetector  # noqa: E402
from mine_shape.geometry import estimate_depth_from_span_px  # noqa: E402
from mine_shape.template import load_template_contour  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
IMAGE_DIR = OUT_DIR / "images"
TEMPLATE = SLAM / "mine_shape" / "templates" / "pfm1_silhouette.png"
PHYSICAL_SPAN_M = 0.120
MIN_SHAPE_CONF = 0.35
MAX_MATCH_DIST = 0.45

SOURCE_ROOTS = [
    Path(r"C:\Users\ozben\Downloads\Part A-20260730T120333Z-1-001\Part A"),
    Path(r"C:\Users\ozben\Downloads\Part B-20260730T120405Z-1-001\Part B"),
    Path(r"C:\Users\ozben\Downloads\Part C-20260730T120425Z-1-001\Part C"),
]

DIST_RE = re.compile(r"(30cm|50cm|1\.5m|1m|2m)", re.IGNORECASE)
DIST_M = {"30cm": 0.30, "50cm": 0.50, "1m": 1.00, "1.5m": 1.50, "2m": 2.00}


@dataclass
class DetOut:
    n: int = 0
    top_conf: float | None = None
    est_depth_m: float | None = None
    span_px: float | None = None
    match_distance: float | None = None
    extras: dict = field(default_factory=dict)


@dataclass
class ImageResult:
    name: str
    part: str
    true_distance_m: float | None
    has_tag_label: bool | None
    stock_shape: DetOut
    diag_shape: DetOut
    blue_shape: DetOut
    tag_n: int
    tag_ids: list[int]
    tag_top_conf: float | None
    tag_est_depth_m: float | None
    aruco_n: int
    notes: str = ""


def is_image_name(name: str) -> bool:
    low = name.lower()
    return low.endswith((".jpg", ".jpeg", ".png", ".webp", ".bmp"))


def dest_name_for(src_name: str) -> str:
    if is_image_name(src_name):
        return src_name
    # "A4_with_tag_1.5m" contains a dot but is not an extension
    return f"{src_name}.jpg"


def copy_images() -> list[Path]:
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    copied: list[Path] = []
    for root in SOURCE_ROOTS:
        if not root.exists():
            print(f"WARN missing source dir: {root}")
            continue
        for src in sorted(root.iterdir()):
            if not src.is_file():
                continue
            dest = IMAGE_DIR / dest_name_for(src.name)
            if not dest.exists() or dest.stat().st_size != src.stat().st_size:
                shutil.copy2(src, dest)
            copied.append(dest)
    return copied


def parse_meta(path: Path) -> tuple[str, float | None, bool | None]:
    stem = path.stem
    # If we wrongly treated ".5m" as extension, stem may be "A4_with_tag_1"
    name = path.name
    part = name[0].upper() if name else "?"
    true_d = None
    m = DIST_RE.search(name)
    if m:
        true_d = DIST_M[m.group(1).lower()]
    has_tag = None
    low = name.lower()
    if "with_tag" in low:
        has_tag = True
    elif "no_tag" in low:
        has_tag = False
    return part, true_d, has_tag


def synth_calibration(image_bgr: np.ndarray) -> CameraCalibration:
    h, w = image_bgr.shape[:2]
    fov_h_deg = 65.0
    fx = (w / 2.0) / np.tan(np.deg2rad(fov_h_deg) / 2.0)
    fy = fx
    cx, cy = w / 2.0, h / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    return CameraCalibration(
        camera_matrix=K,
        dist_coeffs=np.zeros((1, 5), dtype=np.float64),
        image_size=(w, h),
        camera_params=(float(fx), float(fy), float(cx), float(cy)),
    )


def identity_T() -> np.ndarray:
    return np.eye(4, dtype=np.float64)


def load_bgr(path: Path) -> np.ndarray | None:
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is not None:
        return img
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def match_conf(match_distance: float) -> float:
    if match_distance >= MAX_MATCH_DIST:
        return 0.0
    return max(0.0, min(1.0, 1.0 - match_distance / MAX_MATCH_DIST))


def score_contours(
    contours,
    template,
    *,
    min_area: float,
    max_area: float,
    use_ratio_gate: bool,
    template_area: float,
    fx: float,
) -> list[tuple[float, float, float, float, tuple[float, float]]]:
    """Return list of (conf, match_dist, area, span_px, center)."""
    out = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area or area > max_area:
            continue
        if use_ratio_gate and template_area > 0:
            ratio = area / template_area
            if ratio < 0.15 or ratio > 6.0:
                continue
        md = float(cv2.matchShapes(contour, template, cv2.CONTOURS_MATCH_I1, 0.0))
        conf = match_conf(md)
        if conf < MIN_SHAPE_CONF:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6:
            continue
        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        _x, _y, bw, bh = cv2.boundingRect(contour)
        span = float(max(bw, bh))
        out.append((conf, md, float(area), span, (float(cx), float(cy))))
    out.sort(key=lambda t: t[0], reverse=True)
    return out


def to_det_out(scored, fx: float) -> DetOut:
    if not scored:
        return DetOut()
    conf, md, area, span, _c = scored[0]
    depth = estimate_depth_from_span_px(span, fx, PHYSICAL_SPAN_M)
    return DetOut(
        n=len(scored),
        top_conf=conf,
        est_depth_m=depth,
        span_px=span,
        match_distance=md,
        extras={"area_px": area},
    )


def blue_mask(bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Bright cyan/blue plastic under daylight
    m1 = cv2.inRange(hsv, (85, 40, 40), (140, 255, 255))
    m2 = cv2.inRange(hsv, (95, 30, 30), (130, 255, 255))
    mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return mask


def detect_aruco(gray: np.ndarray) -> int:
    try:
        aruco = cv2.aruco
    except AttributeError:
        return -1
    n = 0
    for name in ("DICT_4X4_50", "DICT_5X5_50", "DICT_6X6_50", "DICT_7X7_50", "DICT_ARUCO_ORIGINAL"):
        if not hasattr(aruco, name):
            continue
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, name))
        params = aruco.DetectorParameters()
        if hasattr(aruco, "ArucoDetector"):
            det = aruco.ArucoDetector(dictionary, params)
            _c, ids, _r = det.detectMarkers(gray)
        else:
            _c, ids, _r = aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is not None:
            n = max(n, int(len(ids)))
    return n


def run_one(path: Path, template_contour, template_area: float) -> ImageResult:
    part, true_d, has_tag = parse_meta(path)
    img = load_bgr(path)
    if img is None:
        empty = DetOut()
        return ImageResult(path.name, part, true_d, has_tag, empty, empty, empty, 0, [], None, None, 0, "FAILED_LOAD")

    calib = synth_calibration(img)
    fx = calib.camera_params[0]
    T = identity_T()

    # --- Stock detector (unchanged class logic) ---
    stock_det = ShapeMineDetector(
        calibration=calib,
        template_path=TEMPLATE,
        physical_span_m=PHYSICAL_SPAN_M,
        min_shape_confidence=MIN_SHAPE_CONF,
        max_match_distance=MAX_MATCH_DIST,
        world_drone_transform_provider=lambda _t: T,
        drone_camera_transform=T,
    )
    stock_cands = stock_det.detect(img)
    if stock_cands:
        top = stock_cands[0]
        stock = DetOut(
            n=len(stock_cands),
            top_conf=float(top.confidence),
            est_depth_m=estimate_depth_from_span_px(top.apparent_span_px, fx, PHYSICAL_SPAN_M),
            span_px=float(top.apparent_span_px),
            match_distance=float(top.match_distance),
        )
    else:
        stock = DetOut()

    # --- Diagnostic: same Canny path, ratio gate removed, morph-close edges ---
    und = cv2.undistort(img, calib.camera_matrix, calib.dist_coeffs)
    gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 40, 120)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    diag_scored = score_contours(
        contours,
        template_contour,
        min_area=200.0,
        max_area=5_000_000.0,
        use_ratio_gate=False,
        template_area=template_area,
        fx=fx,
    )
    diag = to_det_out(diag_scored, fx)

    # --- Diagnostic: blue HSV filled silhouette + matchShapes ---
    mask = blue_mask(und)
    blue_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_scored = score_contours(
        blue_contours,
        template_contour,
        min_area=500.0,
        max_area=5_000_000.0,
        use_ratio_gate=False,
        template_area=template_area,
        fx=fx,
    )
    blue = to_det_out(blue_scored, fx)

    # AprilTag (pipeline) + ArUco diagnostic
    tag_det = AprilTagDetector(
        calibration=calib,
        tag_family="tag36h11",
        tag_size_m=0.0381,
        min_confidence=0.05,
    )
    tags = tag_det.detect(img)
    aruco_n = detect_aruco(gray)
    tag_depth = float(np.linalg.norm(tags[0].translation_camera)) if tags else None

    notes = []
    if stock.n == 0 and (diag.n > 0 or blue.n > 0):
        notes.append("stock_gated_out")
    if aruco_n < 0:
        notes.append("aruco_unavailable")
        aruco_n = 0

    return ImageResult(
        name=path.name,
        part=part,
        true_distance_m=true_d,
        has_tag_label=has_tag,
        stock_shape=stock,
        diag_shape=diag,
        blue_shape=blue,
        tag_n=len(tags),
        tag_ids=[int(t.tag_id) for t in tags],
        tag_top_conf=None if not tags else float(tags[0].confidence),
        tag_est_depth_m=tag_depth,
        aruco_n=aruco_n,
        notes=",".join(notes),
    )


def yn(ok: bool) -> str:
    return "Y" if ok else "N"


def fmt(v: float | None, nd: int = 3) -> str:
    if v is None:
        return "  nan"
    return f"{v:{nd+3}.{nd}f}"


def main() -> int:
    if not TEMPLATE.exists():
        print(f"Missing template: {TEMPLATE}")
        return 1

    paths = copy_images()
    template_contour = load_template_contour(TEMPLATE)
    template_area = float(cv2.contourArea(template_contour))

    print(f"Images: {len(paths)} under {IMAGE_DIR}")
    print(f"Template area_px={template_area:.0f}")
    print("Tag path: AprilTag tag36h11 ONLY (pipeline). Fake paper codes may be ArUco/QR.")
    print("Calibration: SYNTHETIC ~65 deg HFOV (no phone calib in repo) — ranging absolute scale untrusted.")
    print(f"pfm_span={PHYSICAL_SPAN_M} m  min_shape_conf={MIN_SHAPE_CONF}")
    print()

    results = [run_one(p, template_contour, template_area) for p in paths]
    (OUT_DIR / "results.json").write_text(
        json.dumps([asdict(r) for r in results], indent=2),
        encoding="utf-8",
    )

    part_a = sorted(
        [r for r in results if r.part == "A"],
        key=lambda r: (r.true_distance_m or 99.0, r.name),
    )

    print("=" * 88)
    print("PART A — Detection vs distance")
    print("=" * 88)
    print(
        f"{'file':<26} {'true':>5} {'stock':>5} {'s_cf':>5} "
        f"{'diag':>5} {'d_cf':>5} {'blue':>5} {'b_cf':>5} "
        f"{'b_est':>6} {'err':>6} {'tag':>4} {'aruco':>5}"
    )
    for r in part_a:
        err = None
        if r.blue_shape.est_depth_m is not None and r.true_distance_m is not None:
            err = r.blue_shape.est_depth_m - r.true_distance_m
        print(
            f"{r.name:<26} {fmt(r.true_distance_m,2)} "
            f"{yn(r.stock_shape.n>0):>5} {fmt(r.stock_shape.top_conf,2)} "
            f"{yn(r.diag_shape.n>0):>5} {fmt(r.diag_shape.top_conf,2)} "
            f"{yn(r.blue_shape.n>0):>5} {fmt(r.blue_shape.top_conf,2)} "
            f"{fmt(r.blue_shape.est_depth_m,2)} {fmt(err,2)} "
            f"{yn(r.tag_n>0):>4} {r.aruco_n:5d}"
        )

    print("\nBy distance (blue diagnostic used for ranging column):")
    print(f"{'dist':>5} {'stock':>8} {'diag':>8} {'blue':>8} {'tagW':>8}")
    dists = sorted({r.true_distance_m for r in part_a if r.true_distance_m is not None})
    for d in dists:
        g = [r for r in part_a if r.true_distance_m == d]
        wt = [r for r in g if r.has_tag_label]
        print(
            f"{d:5.2f} "
            f"{sum(r.stock_shape.n>0 for r in g):3d}/{len(g):<3d} "
            f"{sum(r.diag_shape.n>0 for r in g):3d}/{len(g):<3d} "
            f"{sum(r.blue_shape.n>0 for r in g):3d}/{len(g):<3d} "
            f"{sum(r.tag_n>0 for r in wt):3d}/{len(wt):<3d}"
        )

    print("\nRange accuracy (blue diagnostic depth vs filename truth):")
    ranged = [
        r for r in part_a
        if r.blue_shape.est_depth_m is not None and r.true_distance_m is not None
    ]
    for r in ranged:
        err = r.blue_shape.est_depth_m - r.true_distance_m
        ratio = r.blue_shape.est_depth_m / r.true_distance_m
        print(
            f"  {r.name:<26} true={r.true_distance_m:.2f} est={r.blue_shape.est_depth_m:.3f} "
            f"err={err:+.3f} ratio={ratio:.3f} span_px={r.blue_shape.span_px:.1f}"
        )
    if ranged:
        ratios = [r.blue_shape.est_depth_m / r.true_distance_m for r in ranged]
        errs = [r.blue_shape.est_depth_m - r.true_distance_m for r in ranged]
        print(f"  mean_err={np.mean(errs):+.3f} m  mean_est/true={np.mean(ratios):.3f}")

    print("\n" + "=" * 88)
    print("PART B — Angle robustness")
    print("=" * 88)
    for r in sorted([r for r in results if r.part == "B"], key=lambda x: x.name):
        print(
            f"{r.name:<44} stock={yn(r.stock_shape.n>0)}/{fmt(r.stock_shape.top_conf,2)} "
            f"diag={yn(r.diag_shape.n>0)}/{fmt(r.diag_shape.top_conf,2)} "
            f"blue={yn(r.blue_shape.n>0)}/{fmt(r.blue_shape.top_conf,2)} "
            f"tag={yn(r.tag_n>0)} aruco={r.aruco_n}"
        )

    print("\n" + "=" * 88)
    print("PART C — Decoys / false alarms")
    print("=" * 88)
    for r in sorted([r for r in results if r.part == "C"], key=lambda x: x.name):
        print(
            f"{r.name:<44} stock_n={r.stock_shape.n} diag_n={r.diag_shape.n} "
            f"blue_n={r.blue_shape.n} blue_conf={fmt(r.blue_shape.top_conf,2)} "
            f"tag_n={r.tag_n} ids={r.tag_ids} aruco={r.aruco_n}"
        )

    print("\n" + "=" * 88)
    print("ANSWERS")
    print("=" * 88)

    def max_hit(getter) -> str:
        ok = [d for d in dists if any(getter(r) for r in part_a if r.true_distance_m == d)]
        return f"{max(ok):.2f} m" if ok else "NONE in 0.3-2.0 m set"

    print(f"(1) Max reliable SHAPE distance:")
    print(f"    stock detector:     {max_hit(lambda r: r.stock_shape.n > 0)}")
    print(f"    diag (no ratio):    {max_hit(lambda r: r.diag_shape.n > 0)}")
    print(f"    blue HSV+shape:     {max_hit(lambda r: r.blue_shape.n > 0)}")
    print(f"(2) Max AprilTag tag36h11 decode (with_tag imgs): "
          f"{max_hit(lambda r: r.has_tag_label and r.tag_n > 0)}")
    print("    FLAG: pipeline is AprilTag-only; fake paper codes look ArUco/QR-like — "
          "tag path will not decode them as mines unless they are tag36h11.")

    if ranged:
        ratios = [r.blue_shape.est_depth_m / r.true_distance_m for r in ranged]
        print(
            f"(3) Ranging with span=0.120 m + synth fx: mean est/true={np.mean(ratios):.3f}. "
            "Absolute depths are UNCALIBRATED (phone intrinsics missing)."
        )
    else:
        print("(3) No blue detections for ranging assessment.")

    c3 = [r for r in results if r.name.upper().startswith("C3")]
    c3_stock = all(r.stock_shape.n == 0 and r.tag_n == 0 for r in c3)
    c3_blue = all(r.blue_shape.n == 0 for r in c3)
    print(f"(4) C3 fake-code-only reject: stock+AprilTag clean={c3_stock}; blue_shape clean={c3_blue}")
    for r in c3:
        print(f"    {r.name}: stock={r.stock_shape.n} blue={r.blue_shape.n} tag={r.tag_n} aruco={r.aruco_n}")

    c1 = [r for r in results if r.name.upper().startswith("C1")]
    print(
        f"(5) Stock detector now proposes candidates from a chromatic-blue mask and accepts on "
        f"matchShapes + silhouette IoU (template_area~{template_area:.0f}px is no longer compared "
        f"to contour area). The 'diag'/'blue' columns remain harness-only baselines: diag often "
        f"latches full-frame border contours, and blue alone still false-fires on C3. "
        f"C1 blue multi-cand={sum(r.blue_shape.n>1 for r in c1)}/{len(c1)}."
    )
    print("NOTE: no photos >2 m — high-altitude coarse search uncharacterized.")
    print(f"Wrote {OUT_DIR / 'results.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
