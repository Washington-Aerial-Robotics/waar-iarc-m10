from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np

try:
    import trimesh
except ImportError as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit(
        "Missing dependency: trimesh. Install with `pip install trimesh`."
    ) from exc


def rotation_matrix(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    axis = axis.astype(np.float64)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        raise ValueError("Rotation axis is near-zero; cannot build matrix.")
    axis /= norm
    angle = math.radians(angle_deg)
    c = math.cos(angle)
    s = math.sin(angle)
    x, y, z = axis
    return np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ],
        dtype=np.float64,
    )


def project_and_fill(
    vertices_mm: np.ndarray,
    faces: np.ndarray,
    *,
    border_px: int = 6,
    long_dim_px: int = 512,
    supersample: int = 4,
) -> tuple[np.ndarray, tuple[float, float]]:
    verts2 = vertices_mm[:, :2]
    min_xy = verts2.min(axis=0)
    max_xy = verts2.max(axis=0)
    span_xy_mm = max_xy - min_xy

    if span_xy_mm[0] <= 0.0 or span_xy_mm[1] <= 0.0:
        raise ValueError(f"Degenerate projected span: {span_xy_mm}")

    long_mm = float(max(span_xy_mm[0], span_xy_mm[1]))
    px_per_mm = float(long_dim_px) / long_mm

    canvas_w = int(math.ceil(span_xy_mm[0] * px_per_mm)) + 2 * border_px
    canvas_h = int(math.ceil(span_xy_mm[1] * px_per_mm)) + 2 * border_px
    canvas_w = max(canvas_w, 8)
    canvas_h = max(canvas_h, 8)

    hi_w = canvas_w * supersample
    hi_h = canvas_h * supersample
    hi = np.zeros((hi_h, hi_w), dtype=np.uint8)

    scale = px_per_mm * supersample
    offset = np.array([border_px * supersample, border_px * supersample], dtype=np.float64)

    projected = (verts2 - min_xy) * scale + offset
    projected[:, 1] = hi_h - 1 - projected[:, 1]

    for tri in faces:
        poly = projected[tri].astype(np.int32)
        cv2.fillConvexPoly(hi, poly, 255, lineType=cv2.LINE_AA)

    lo = cv2.resize(hi, (canvas_w, canvas_h), interpolation=cv2.INTER_AREA)
    silhouette = np.where(lo > 4, 255, 0).astype(np.uint8)

    ys, xs = np.where(silhouette > 0)
    if len(xs) == 0 or len(ys) == 0:
        raise ValueError("Rasterization produced an empty silhouette.")

    x0 = max(int(xs.min()) - border_px, 0)
    x1 = min(int(xs.max()) + border_px + 1, silhouette.shape[1])
    y0 = max(int(ys.min()) - border_px, 0)
    y1 = min(int(ys.max()) + border_px + 1, silhouette.shape[0])
    tight = silhouette[y0:y1, x0:x1]
    return tight, (float(span_xy_mm[0]), float(span_xy_mm[1]))


def render_view(mesh: trimesh.Trimesh, pitch_deg: float) -> tuple[np.ndarray, tuple[float, float]]:
    verts = mesh.vertices.astype(np.float64).copy()
    center = verts.mean(axis=0)
    verts -= center

    spans = verts.max(axis=0) - verts.min(axis=0)
    short_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if spans[1] < spans[0]:
        short_axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    if abs(pitch_deg) > 1e-9:
        rot = rotation_matrix(short_axis, pitch_deg)
        verts = verts @ rot.T

    return project_and_fill(verts, mesh.faces)


def load_mesh(stl_path: Path) -> trimesh.Trimesh:
    if not stl_path.exists():
        raise FileNotFoundError(f"STL not found: {stl_path}")

    mesh = trimesh.load_mesh(stl_path, force="mesh")
    if mesh is None:
        raise RuntimeError(f"Failed to load mesh from: {stl_path}")
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError(f"Expected a single mesh, got: {type(mesh).__name__}")
    if mesh.vertices.size == 0 or mesh.faces.size == 0:
        raise RuntimeError("Mesh is empty (no vertices/faces).")

    bounds = mesh.bounds
    spans = bounds[1] - bounds[0]
    if np.any(spans <= 1e-6):
        raise RuntimeError(f"Mesh bounds are degenerate: {spans}")

    return mesh


def write_png(path: Path, img: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(path), img)
    if not ok:
        raise RuntimeError(f"Failed to write PNG: {path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Render PFM-1 silhouette templates from STL (orthographic, filled)."
    )
    parser.add_argument(
        "--stl",
        type=Path,
        required=True,
        help="Path to IARC_PFM-1_mine.stl (millimeter units).",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "templates",
        help="Output directory for template PNGs.",
    )
    parser.add_argument(
        "--include-pitched",
        action="store_true",
        help="Also render 30deg and 45deg pitched orthographic silhouettes.",
    )
    args = parser.parse_args()

    mesh = load_mesh(args.stl)
    spans_mm = mesh.bounds[1] - mesh.bounds[0]
    print(
        "Loaded mesh: "
        f"x={spans_mm[0]:.2f} mm, y={spans_mm[1]:.2f} mm, z={spans_mm[2]:.2f} mm, "
        f"faces={len(mesh.faces)}"
    )

    top_img, (sx, sy) = render_view(mesh, pitch_deg=0.0)
    top_path = args.out_dir / "pfm1_silhouette.png"
    write_png(top_path, top_img)
    print(
        f"Wrote {top_path} | pixels={top_img.shape[1]}x{top_img.shape[0]} | "
        f"projected_span_mm=({sx:.2f}, {sy:.2f}) | max_span_mm={max(sx, sy):.2f}"
    )

    if args.include_pitched:
        for deg in (30.0, 45.0):
            img, (px, py) = render_view(mesh, pitch_deg=deg)
            out_path = args.out_dir / f"pfm1_silhouette_{int(deg)}deg.png"
            write_png(out_path, img)
            print(
                f"Wrote {out_path} | pixels={img.shape[1]}x{img.shape[0]} | "
                f"projected_span_mm=({px:.2f}, {py:.2f}) | max_span_mm={max(px, py):.2f}"
            )


if __name__ == "__main__":
    main()
