from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2

SLAM_DIR = Path(__file__).resolve().parent
if str(SLAM_DIR) not in sys.path:
    sys.path.insert(0, str(SLAM_DIR))

from apriltag import (  # noqa: E402
    AprilTagDetector,
    CameraSource,
    DetectionCsvLogger,
    MineRegistry,
    PipelineConfig,
    PoseSource,
    create_pose_provider,
    homogeneous_from_pose,
    is_headless,
    load_calibration,
    split_stereo_frame,
    tag_pose_to_world,
)
from mine_shape import (  # noqa: E402
    ShapeMineDetector,
    ShapeMineRegistry,
    filter_shapes_away_from_tags,
    remove_shapes_near_tag_world,
)
from sparse_voxel_map import SparseVoxelMap  # noqa: E402


class PerceptionPipeline:
    """AprilTag + classical PFM-1 shape mine detection, mapping, and localization."""

    def __init__(self, config: PipelineConfig | None = None):
        self.config = config or PipelineConfig.from_json()
        if is_headless() and self.config.enable_visualization:
            print("Headless environment detected — disabling visualization.")
            self.config.enable_visualization = False

        self.calibration = load_calibration(self.config.calib_file)
        if self.config.stereo_fx is not None and self.config.camera_mode == "stereo":
            pass  # reserved for future stereo depth / SLAM obstacle layer

        self.detector = AprilTagDetector(
            calibration=self.calibration,
            tag_family=self.config.tag_family,
            tag_size_m=self.config.tag_size_m,
            min_confidence=self.config.min_confidence,
            use_rmse_confidence=self.config.use_rmse_confidence,
        )
        self.pose_provider = create_pose_provider(self.config)
        self.drone_camera_transform = homogeneous_from_pose(
            self.config.drone_camera_position,
            self.config.drone_camera_quaternion,
        )
        self.mine_registry = MineRegistry(
            min_confidence=self.config.min_confidence,
            use_kalman=self.config.use_kalman_fusion,
        )
        self.shape_registry = ShapeMineRegistry(
            min_confidence=self.config.min_shape_confidence,
            fusion_radius_m=self.config.shape_fusion_radius_m,
            use_kalman=self.config.use_kalman_fusion,
        )
        self.shape_detector: ShapeMineDetector | None = None
        if self.config.enable_shape_detection:
            self.shape_detector = ShapeMineDetector(
                calibration=self.calibration,
                template_path=self.config.shape_template_path,
                physical_span_m=self.config.pfm_physical_span_m,
                min_shape_confidence=self.config.min_shape_confidence,
                max_match_distance=self.config.shape_max_match_distance,
                min_contour_area_px=self.config.shape_min_contour_area_px,
                max_contour_area_px=self.config.shape_max_contour_area_px,
                canny_low=self.config.shape_canny_low,
                canny_high=self.config.shape_canny_high,
                morph_kernel=self.config.shape_morph_kernel,
                min_span_px=self.config.shape_min_span_px,
                max_span_px=self.config.shape_max_span_px,
                min_aspect=self.config.shape_min_aspect,
                max_aspect=self.config.shape_max_aspect,
                min_solidity=self.config.shape_min_solidity,
                min_extent=self.config.shape_min_extent,
                min_silhouette_iou=self.config.shape_min_silhouette_iou,
                use_chromatic_proposal=self.config.shape_use_chromatic_proposal,
                blue_dom_margin=self.config.shape_blue_dom_margin,
                blue_hue_min=self.config.shape_blue_hue_min,
                blue_hue_max=self.config.shape_blue_hue_max,
                blue_min_sat=self.config.shape_blue_min_sat,
                blue_min_value=self.config.shape_blue_min_value,
                ground_z_m=self.config.ground_z_m,
                world_drone_transform_provider=self.pose_provider.world_drone_transform,
                drone_camera_transform=self.drone_camera_transform,
            )
            if self.shape_detector.template_ready:
                print("[mine_shape] PFM-1 template loaded — shape branch enabled")
            else:
                print("[mine_shape] no template — shape branch idle (AprilTags only)")

        self.voxel_map = SparseVoxelMap(
            {"x": 0.0, "y": 0.0, "z": 0.0},
            resolution=self.config.map_resolution,
            field_x=self.config.field_x,
            field_y=self.config.field_y,
        )

        self.camera = CameraSource(
            camera_index=self.config.camera_index,
            request_width=self.config.request_width,
            request_height=self.config.request_height,
            use_v4l2=self.config.use_v4l2,
            force_mjpeg=self.config.force_mjpeg,
        )
        self.csv_logger = (
            DetectionCsvLogger(self.config.csv_log_file)
            if self.config.enable_csv_log
            else None
        )
        self._running = True
        self._last_world_positions = {}
        self._last_shape_candidates = []

    def stop(self) -> None:
        self._running = False

    def process_mine_detections(self, detections, timestamp: float) -> list:
        world_drone = self.pose_provider.world_drone_transform(timestamp)
        drone_pose = self.pose_provider.get_pose(timestamp)
        self.voxel_map.update_drone_position(drone_pose.position)

        updated_mines = []
        world_positions = {}
        mine_ids = set(self.config.mine_tag_ids)
        for detection in detections:
            if detection.tag_id not in mine_ids:
                # Decoy / non-mine arena object — decode OK, do not register as a mine.
                continue

            world_position, world_rotation = tag_pose_to_world(
                detection.translation_camera,
                detection.rotation_camera,
                world_drone,
                self.drone_camera_transform,
            )
            world_positions[detection.tag_id] = world_position

            fused = self.mine_registry.update(
                tag_id=detection.tag_id,
                world_position=world_position,
                world_rotation=world_rotation,
                confidence=detection.confidence,
                timestamp=timestamp,
            )
            if fused is None:
                continue

            remove_shapes_near_tag_world(
                self.shape_registry,
                world_position,
                radius_m=self.config.shape_dedupe_radius_m,
            )

            if (
                fused.observation_count > 1
                and hasattr(self.pose_provider, "apply_tag_correction")
            ):
                self.pose_provider.apply_tag_correction(
                    world_position,
                    fused.world_position,
                    detection.confidence,
                )

            self.voxel_map.add_mine_world(
                float(fused.world_position[0]),
                float(fused.world_position[1]),
            )
            updated_mines.append(fused)

            if self.csv_logger is not None:
                self.csv_logger.log(detection, world_position, fused)

        self._last_world_positions = world_positions
        return updated_mines

    def _tag_world_positions_for_dedupe(self) -> list:
        positions = list(self._last_world_positions.values())
        for mine in self.mine_registry.mines.values():
            if mine.tag_id is not None:
                positions.append(mine.world_position)
        return positions

    def process_shape_detections(self, candidates, timestamp: float) -> list:
        if not candidates:
            self._last_shape_candidates = []
            return []

        filtered = filter_shapes_away_from_tags(
            candidates,
            self._tag_world_positions_for_dedupe(),
            radius_m=self.config.shape_dedupe_radius_m,
        )
        self._last_shape_candidates = filtered

        updated = []
        for cand in filtered:
            fused = self.shape_registry.update(
                cand.world_position,
                cand.confidence,
                timestamp,
            )
            if fused is None:
                continue
            self.voxel_map.add_mine_world(
                float(fused.world_position[0]),
                float(fused.world_position[1]),
            )
            updated.append(fused)
        return updated

    def _log_stats(
        self,
        stats_file: Path,
        frame_count: int,
        elapsed: float,
        detect_ms: float,
    ) -> None:
        fps = frame_count / elapsed if elapsed > 0 else 0.0
        pose_extra = ""
        if hasattr(self.pose_provider, "stats"):
            stats = self.pose_provider.stats
            pos = stats["position"]
            pose_extra = (
                f" pose=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})"
                f" corrections={stats.get('correction_count', 0)}"
            )
        line = (
            f"frames={frame_count} elapsed={elapsed:.1f}s fps={fps:.1f} "
            f"last_detect_ms={detect_ms:.1f} "
            f"tag_mines={len(self.mine_registry.mines)} "
            f"shape_mines={len(self.shape_registry.mines)}{pose_extra}"
        )
        print(f"[stats] {line}")
        if self.config.enable_stats_log:
            with stats_file.open("a", encoding="utf-8") as f:
                f.write(f"{time.time():.3f} {line}\n")

    def _save_maps(self) -> None:
        if self.mine_registry.mines or self.shape_registry.mines:
            self.voxel_map.generate_visualization()

    def run(self) -> None:
        self.camera.open()
        width, height = self.camera.actual_resolution
        print(f"Camera resolution: {width} x {height}")
        print(f"Camera mode: {self.config.camera_mode}")
        print(f"Calibration: {self.config.calib_file}")
        print(f"Pose source: {self.config.pose_source.value}")
        if self.config.enable_visualization:
            print("Press 'q' to quit")
        else:
            print("Running headless — Ctrl+C to quit")

        if self.calibration.image_size is not None:
            calib_w, calib_h = self.calibration.image_size
            if (calib_w, calib_h) != (width, height) and self.config.camera_mode == "mono":
                print(
                    f"Warning: runtime resolution {width}x{height} differs from "
                    f"calibration resolution {calib_w}x{calib_h}"
                )

        frame_count = 0
        start_time = time.time()
        last_stats_time = start_time
        last_detect_ms = 0.0
        stats_file = self.config.stats_log_file

        if self.config.enable_stats_log:
            stats_file.write_text("", encoding="utf-8")

        try:
            while self._running:
                ok, frame = self.camera.read()
                if not ok:
                    print("Failed to read camera frame")
                    break

                timestamp = time.time()
                t0 = time.perf_counter()

                if self.config.camera_mode == "stereo":
                    stereo = split_stereo_frame(frame)
                    if stereo is None:
                        print("Expected stereo frame but could not split eyes — skipping frame")
                        continue
                    mine_frame, _right, _eye_w = stereo
                else:
                    mine_frame = frame

                if hasattr(self.pose_provider, "update_frame"):
                    self.pose_provider.update_frame(mine_frame, timestamp=timestamp)

                mine_detections = self.detector.detect(mine_frame, timestamp=timestamp)
                fused_mines = self.process_mine_detections(mine_detections, timestamp)

                fused_shapes = []
                if self.shape_detector is not None:
                    shape_candidates = self.shape_detector.detect(mine_frame, timestamp=timestamp)
                    fused_shapes = self.process_shape_detections(shape_candidates, timestamp)

                last_detect_ms = (time.perf_counter() - t0) * 1000.0

                for mine in fused_mines:
                    pos = mine.world_position
                    print(
                        f"Mine tag {mine.tag_id}: "
                        f"world=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                        f"conf={mine.confidence:.2f} obs={mine.observation_count}"
                    )
                for mine in fused_shapes:
                    pos = mine.world_position
                    print(
                        f"Mine shape {mine.shape_id}: "
                        f"world=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                        f"conf={mine.confidence:.2f} obs={mine.observation_count} (pending tag)"
                    )

                if self.config.enable_visualization:
                    display = self.detector.draw_detections(
                        mine_frame,
                        mine_detections,
                        world_positions=self._last_world_positions,
                    )
                    if self.shape_detector is not None and self._last_shape_candidates:
                        display = self.shape_detector.draw_candidates(
                            display,
                            self._last_shape_candidates,
                        )
                    cv2.imshow("Perception Pipeline", display)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                frame_count += 1
                now = time.time()
                if now - last_stats_time >= self.config.stats_interval_s:
                    self._log_stats(stats_file, frame_count, now - start_time, last_detect_ms)
                    last_stats_time = now

                if frame_count % self.config.map_save_interval_frames == 0:
                    self._save_maps()
        finally:
            self.camera.release()
            if hasattr(self.pose_provider, "close"):
                self.pose_provider.close()
            if self.config.enable_visualization:
                cv2.destroyAllWindows()
            if self.csv_logger is not None:
                self.csv_logger.close()

        print(f"\nDiscovered tag mines: {len(self.mine_registry.mines)}")
        for tag_id, mine in sorted(self.mine_registry.mines.items()):
            pos = mine.world_position
            print(
                f"  tag {tag_id}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                f"observations={mine.observation_count}"
            )
        print(f"Shape-only candidates: {len(self.shape_registry.mines)}")
        for shape_id, mine in sorted(self.shape_registry.mines.items()):
            pos = mine.world_position
            print(
                f"  shape {shape_id}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                f"observations={mine.observation_count} conf={mine.confidence:.2f}"
            )

        self._save_maps()
        print("Saved map visualization to occ_grid_proj.png")


MineDetectionPipeline = PerceptionPipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="IARC perception pipeline (AprilTag mines + classical PFM-1 shape)"
    )
    parser.add_argument("--config", type=Path, default=SLAM_DIR / "pipeline_config.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--no-v4l2", action="store_true")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument("--pose-source", choices=["stub", "esp32", "fused"], default=None)
    parser.add_argument("--esp32-host", type=str, default=None)
    parser.add_argument("--calib", type=Path, default=None)
    parser.add_argument(
        "--stereo",
        action="store_true",
        help="Side-by-side stereo camera (left eye used for AprilTags)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = PipelineConfig.from_json(args.config)

    if args.camera_index is not None:
        config.camera_index = args.camera_index
    if args.width is not None:
        config.request_width = args.width
    if args.height is not None:
        config.request_height = args.height
    if args.no_v4l2:
        config.use_v4l2 = False
    if args.headless:
        config.enable_visualization = False
    if args.visualize:
        config.enable_visualization = True
    if args.no_csv:
        config.enable_csv_log = False
    if args.calib is not None:
        config.calib_file = args.calib
    if args.pose_source is not None:
        config.pose_source = PoseSource(args.pose_source)
    if args.esp32_host is not None:
        config.esp32_host = args.esp32_host
    if args.stereo:
        config.camera_mode = "stereo"

    pipeline = PerceptionPipeline(config)

    def _handle_signal(_signum, _frame):
        print("\nStopping pipeline...")
        pipeline.stop()

    signal.signal(signal.SIGINT, _handle_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _handle_signal)

    pipeline.run()


if __name__ == "__main__":
    main()
