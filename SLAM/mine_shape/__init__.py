"""Classical PFM-1 silhouette mine detection (no ML)."""

from .detector import ShapeMineDetector
from .fusion import filter_shapes_away_from_tags, remove_shapes_near_tag_world
from .geometry import shape_center_to_world
from .models import ShapeMineCandidate
from .registry import ShapeMineRegistry
from .template import TemplateLoadError, load_template_contour

__all__ = [
    "ShapeMineCandidate",
    "ShapeMineDetector",
    "ShapeMineRegistry",
    "TemplateLoadError",
    "filter_shapes_away_from_tags",
    "load_template_contour",
    "remove_shapes_near_tag_world",
    "shape_center_to_world",
]
