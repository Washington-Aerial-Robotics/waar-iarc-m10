from __future__ import annotations

import time

import cv2
import numpy as np

from apriltag.kalman import TagTrack

from .geometry import bbox_corners_camera
from .models import FusedObstacle, ObstacleDetection
from .stereo import depth_cluster_detections, depth_range_in_bbox


class ObstacleDetector:
    """YOLO + stereo depth obstacle detection (trees and other configured classes)."""

    def __init__(
        self,
        *,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        baseline_m: float,
        model_path: str = "yolov8n.pt",
        target_classes: list[str] | None = None,
        min_confidence: float = 0.25,
        min_object_area: int = 2000,
        min_depth_m: float = 0.5,
        max_depth_m: float = 8.0,
        detection_mode: str = "both",
        yolo_interval: int = 3,
    ):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.baseline_m = baseline_m
        self.target_classes = {c.lower() for c in (target_classes or ["tree"])}
        self.min_confidence = min_confidence
        self.min_object_area = min_object_area
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        self.detection_mode = detection_mode
        self.yolo_interval = max(1, yolo_interval)
        self._frame_count = 0
        self._model = None
        self._model_path = model_path
        self._stereo = None

    def _ensure_model(self) -> None:
        if self._model is not None:
            return
        if self.detection_mode in ("yolo", "both"):
            from ultralytics import YOLO

            self._model = YOLO(self._model_path)

    def _ensure_stereo(self) -> None:
        if self._stereo is None:
            from .stereo import build_stereo_matcher

            self._stereo = build_stereo_matcher()

    @property
    def stereo_matcher(self):
        self._ensure_stereo()
        return self._stereo

    def detect(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
        timestamp: float | None = None,
    ) -> list[ObstacleDetection]:
        if timestamp is None:
            timestamp = time.time()

        self._frame_count += 1
        left_gray = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)

        from .stereo import compute_depth_map

        depth_map, valid_mask = compute_depth_map(
            left_gray,
            right_gray,
            self.stereo_matcher,
            self.fx,
            self.baseline_m,
            self.min_depth_m,
            self.max_depth_m,
        )

        candidates: list[tuple[int, int, int, int, str, float, float, float]] = []

        if self.detection_mode in ("yolo", "both") and self._frame_count % self.yolo_interval == 0:
            self._ensure_model()
            if self._model is not None:
                for x, y, w, h, label, conf in self._detect_yolo(left_bgr):
                    depth_range = depth_range_in_bbox(depth_map, valid_mask, x, y, w, h)
                    if depth_range is None:
                        continue
                    d_near, d_far = depth_range
                    candidates.append((x, y, w, h, label, conf, d_near, d_far))

        if self.detection_mode in ("depth", "both"):
            for x, y, w, h, d_near, d_far in depth_cluster_detections(
                depth_map,
                valid_mask,
                min_area_px=self.min_object_area,
                min_depth_m=self.min_depth_m,
                max_depth_m=self.max_depth_m,
            ):
                candidates.append((x, y, w, h, "obstacle", 0.4, d_near, d_far))

        return self._dedupe_candidates(candidates, timestamp)

    def _detect_yolo(self, left_bgr: np.ndarray) -> list[tuple[int, int, int, int, str, float]]:
        results = self._model(left_bgr, verbose=False, conf=self.min_confidence)
        boxes: list[tuple[int, int, int, int, str, float]] = []
        for result in results:
            for box in result.boxes:
                cls_id = int(box.cls[0])
                label = str(self._model.names[cls_id]).lower()
                if self.target_classes and label not in self.target_classes:
                    continue
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                h = y2 - y1
                if w * h < self.min_object_area:
                    continue
                boxes.append((x1, y1, w, h, label, conf))
        return boxes

    def _dedupe_candidates(
        self,
        candidates: list[tuple[int, int, int, int, str, float, float, float]],
        timestamp: float,
    ) -> list[ObstacleDetection]:
        """Merge overlapping bboxes, preferring higher-confidence YOLO labels."""
        if not candidates:
            return []

        candidates.sort(key=lambda c: c[5], reverse=True)
        kept: list[tuple[int, int, int, int, str, float, float, float]] = []
        for cand in candidates:
            x, y, w, h, label, conf, d_near, d_far = cand
            cx = x + w / 2
            cy = y + h / 2
            duplicate = False
            for kx, ky, kw, kh, klabel, kconf, _, _ in kept:
                kcx = kx + kw / 2
                kcy = ky + kh / 2
                if abs(cx - kcx) < min(w, kw) * 0.5 and abs(cy - kcy) < min(h, kh) * 0.5:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(cand)

        detections: list[ObstacleDetection] = []
        for x, y, w, h, label, conf, d_near, d_far in kept:
            corners = bbox_corners_camera(x, y, w, h, d_near, d_far, self.fx, self.fy, self.cx, self.cy)
            detections.append(
                ObstacleDetection(
                    timestamp=timestamp,
                    label=label,
                    confidence=conf,
                    bbox_xywh=(x, y, w, h),
                    depth_near_m=d_near,
                    depth_far_m=d_far,
                    corners_camera=corners,
                )
            )
        return detections

    def draw_detections(self, image: np.ndarray, detections: list[ObstacleDetection]) -> np.ndarray:
        output = image.copy()
        for det in detections:
            x, y, w, h = det.bbox_xywh
            color = (0, 165, 255) if det.label == "tree" else (0, 128, 255)
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
            cv2.putText(
                output,
                f"{det.label} {det.confidence:.2f} z:{det.depth_near_m:.1f}-{det.depth_far_m:.2f}m",
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        return output


class ObstacleRegistry:
    """Fuse repeated obstacle sightings by proximity in world space."""

    def __init__(
        self,
        fusion_radius_m: float = 1.0,
        min_confidence: float = 0.2,
        use_kalman: bool = True,
    ):
        self.fusion_radius_m = fusion_radius_m
        self.min_confidence = min_confidence
        self.use_kalman = use_kalman
        self._obstacles: dict[int, FusedObstacle] = {}
        self._tracks: dict[int, TagTrack] = {}
        self._next_id = 1

    @property
    def obstacles(self) -> dict[int, FusedObstacle]:
        return dict(self._obstacles)

    def _find_match(self, world_position: np.ndarray) -> int | None:
        best_id = None
        best_dist = self.fusion_radius_m
        for obstacle_id, obstacle in self._obstacles.items():
            dist = float(np.linalg.norm(obstacle.world_position - world_position))
            if dist < best_dist:
                best_dist = dist
                best_id = obstacle_id
        return best_id

    def update(
        self,
        label: str,
        world_position: np.ndarray,
        confidence: float,
        timestamp: float,
        height_m: float,
    ) -> FusedObstacle | None:
        if confidence < self.min_confidence:
            return None

        world_position = world_position.astype(np.float64).reshape(3)
        obstacle_id = self._find_match(world_position)

        if obstacle_id is None:
            obstacle_id = self._next_id
            self._next_id += 1
            fused = FusedObstacle(
                obstacle_id=obstacle_id,
                label=label,
                first_seen=timestamp,
                last_seen=timestamp,
                observation_count=1,
                world_position=world_position.copy(),
                confidence=float(confidence),
                height_m=float(height_m),
            )
            self._obstacles[obstacle_id] = fused
            if self.use_kalman:
                self._tracks[obstacle_id] = TagTrack(world_position, timestamp)
            return fused

        if self.use_kalman and obstacle_id in self._tracks:
            world_position = self._tracks[obstacle_id].update(world_position, timestamp)

        existing = self._obstacles[obstacle_id]
        old_weight = existing.confidence * existing.observation_count
        new_weight = confidence
        total_weight = old_weight + new_weight
        fused_position = (
            existing.world_position * old_weight + world_position * new_weight
        ) / total_weight

        existing.last_seen = timestamp
        existing.observation_count += 1
        existing.world_position = fused_position
        existing.confidence = min(1.0, (existing.confidence + confidence) / 2.0)
        existing.height_m = max(existing.height_m, height_m)
        if label != "obstacle":
            existing.label = label
        return existing

    def export_records(self) -> list[dict]:
        records = []
        for obstacle in self._obstacles.values():
            pos = obstacle.world_position
            records.append(
                {
                    "obstacle_id": obstacle.obstacle_id,
                    "label": obstacle.label,
                    "world_x": float(pos[0]),
                    "world_y": float(pos[1]),
                    "world_z": float(pos[2]),
                    "confidence": float(obstacle.confidence),
                    "height_m": float(obstacle.height_m),
                    "radius_m": float(obstacle.radius_m),
                    "observation_count": obstacle.observation_count,
                }
            )
        return records

    def import_records(self, records: list[dict]) -> int:
        merged = 0
        for record in records:
            pos = np.array(
                [record["world_x"], record["world_y"], record.get("world_z", 0.0)],
                dtype=np.float64,
            )
            confidence = float(record.get("confidence", 0.5))
            label = str(record.get("label", "obstacle"))
            height_m = float(record.get("height_m", 2.0))
            if self.update(label, pos, confidence, time.time(), height_m) is not None:
                merged += 1
        return merged
