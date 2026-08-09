from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

DEFAULT_TEMPLATE_PATH = Path(__file__).resolve().parent / "templates" / "pfm1_silhouette.png"


class TemplateLoadError(FileNotFoundError):
    pass


def load_template_contour(
    template_path: Path | None,
    *,
    canny_low: int = 50,
    canny_high: int = 150,
) -> np.ndarray:
    """
    Load PFM-1 reference silhouette and return the largest closed contour.

    TODO: Add `SLAM/mine_shape/templates/pfm1_silhouette.png` — binary or high-contrast
    image of the PFM-1 outline from the IARC Resource Addendum (winged asymmetric view).
    Recommended: ~256px wide PNG, white silhouette on black background.
    """
    path = template_path or DEFAULT_TEMPLATE_PATH
    if not path.exists():
        raise TemplateLoadError(
            f"PFM-1 template missing: {path}. "
            "Add a silhouette PNG at SLAM/mine_shape/templates/pfm1_silhouette.png"
        )

    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise TemplateLoadError(f"Could not read template image: {path}")

    # White silhouette on black (see templates/README). Fixed threshold — do not
    # invert by mean (that turns a large white body into a black hole whose
    # RETR_EXTERNAL contour is the image border and breaks matchShapes).
    _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    if binary[0, 0] > 0 and binary[0, -1] > 0 and binary[-1, 0] > 0:
        binary = 255 - binary
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours or cv2.contourArea(max(contours, key=cv2.contourArea)) < 50:
        edges = cv2.Canny(cv2.GaussianBlur(img, (5, 5), 0), canny_low, canny_high)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise TemplateLoadError(f"No contours in template: {path}")

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 50:
        raise TemplateLoadError(f"Template contour too small: {path}")
    return contour
