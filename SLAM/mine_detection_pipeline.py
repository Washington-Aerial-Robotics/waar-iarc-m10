from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

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
    tag_pose_to_world,
)
from obstacle import (  # noqa: E402
    ObstacleDetector,
    ObstacleRegistry,
    camera_points_to_world,
    export_obstacle_map,
    import_obstacle_map,
    split_stereo_frame,
)
from sparse_voxel_map import SparseVoxelMap  # noqa: E402


class PerceptionPipeline:
    """Unified mine (AprilTag) + obstacle (tree/stereo) perception pipeline."""

    def __init__(self, config: PipelineConfig | None = None):
        self.config = config or PipelineConfig.from_json()
        if is_headless() and self.config.enable_visualization:
            print("Headless environment detected — disabling visualization.")
            self.config.enable_visualization = False

        self.calibration = load_calibration(self.config.calib_file)
        fx, fy, cx, cy = self.calibration.camera_params
        if self.config.stereo_fx is not None:
            fx = float(self.config.stereo_fx)

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
        self.obstacle_registry: ObstacleRegistry | None = None
        self.obstacle_detector: ObstacleDetector | None = None
        if self.config.enable_obstacles and self.config.camera_mode == "stereo":
            self.obstacle_registry = ObstacleRegistry(
                fusion_radius_m=self.config.obstacle_fusion_radius_m,
                min_confidence=self.config.obstacle_min_confidence,
                use_kalman=self.config.obstacle_use_kalman,
            )
            self.obstacle_detector = ObstacleDetector(
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                baseline_m=self.config.stereo_baseline_m,
                model_path=self.config.obstacle_model,
                target_classes=self.config.obstacle_classes,
                min_confidence=self.config.obstacle_min_confidence,
                min_depth_m=self.config.obstacle_min_depth_m,
                max_depth_m=self.config.obstacle_max_depth_m,
                detection_mode=self.config.obstacle_detection_mode,
            )
            if self.config.shared_obstacle_map_file.exists():
                merged = import_obstacle_map(
                    self.obstacle_registry, self.config.shared_obstacle_map_file
                )
                print(f"Loaded {merged} shared obstacles from {self.config.shared_obstacle_map_file}")
        elif self.config.enable_obstacles:
            print("Warning: obstacles enabled but camera_mode is not 'stereo' — obstacle detection disabled.")

        self.voxel_map = SparseVoxelMap(
            {"x": 0.0, "y": 0.0, "z": 0.0},
            resolution=self.config.map_resolution,
            field_x=self.config.field_x,
            field_y=self.config.field_y,
        )
        self._sync_obstacles_to_map()

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
        self._last_obstacle_detections = []

    def stop(self) -> None:
        self._running = False

    def _sync_obstacles_to_map(self) -> None:
        if self.obstacle_registry is None:
            return
        for obstacle in self.obstacle_registry.obstacles.values():
            pos = obstacle.world_position
            self.voxel_map.add_obstacle_world(
                float(pos[0]),
                float(pos[1]),
                float(pos[2]),
                height_m=obstacle.height_m,
                radius_m=obstacle.radius_m,
            )

    def process_mine_detections(self, detections, timestamp: float) -> list:
        world_drone = self.pose_provider.world_drone_transform(timestamp)
        drone_pose = self.pose_provider.get_pose(timestamp)
        self.voxel_map.update_drone_position(drone_pose.position)

        updated_mines = []
        world_positions = {}
        for detection in detections:
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

            self.voxel_map.add_mine_world(
                float(fused.world_position[0]),
                float(fused.world_position[1]),
            )
            updated_mines.append(fused)

            if self.csv_logger is not None:
                self.csv_logger.log(detection, world_position, fused)

        self._last_world_positions = world_positions
        return updated_mines

    def process_obstacle_detections(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
        timestamp: float,
    ) -> list:
        if self.obstacle_detector is None or self.obstacle_registry is None:
            return []

        world_drone = self.pose_provider.world_drone_transform(timestamp)
        detections = self.obstacle_detector.detect(left_bgr, right_bgr, timestamp=timestamp)
        self._last_obstacle_detections = detections

        updated = []
        for detection in detections:
            world_corners = camera_points_to_world(
                detection.corners_camera,
                world_drone,
                self.drone_camera_transform,
            )
            world_center = world_corners.mean(axis=0)
            height_m = max(
                self.config.obstacle_default_height_m,
                float(detection.depth_far_m - detection.depth_near_m),
            )

            fused = self.obstacle_registry.update(
                label=detection.label,
                world_position=world_center,
                confidence=detection.confidence,
                timestamp=timestamp,
                height_m=height_m,
            )
            if fused is None:
                continue

            fused.radius_m = self.config.obstacle_default_radius_m
            pos = fused.world_position
            self.voxel_map.add_obstacle_world(
                float(pos[0]),
                float(pos[1]),
                float(pos[2]),
                height_m=fused.height_m,
                radius_m=fused.radius_m,
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
        obstacle_count = 0 if self.obstacle_registry is None else len(self.obstacle_registry.obstacles)
        line = (
            f"frames={frame_count} elapsed={elapsed:.1f}s fps={fps:.1f} "
            f"last_detect_ms={detect_ms:.1f} mines={len(self.mine_registry.mines)} "
            f"obstacles={obstacle_count}"
        )
        print(f"[stats] {line}")
        if self.config.enable_stats_log:
            with stats_file.open("a", encoding="utf-8") as f:
                f.write(f"{time.time():.3f} {line}\n")

    def _save_maps(self) -> None:
        if self.mine_registry.mines or (
            self.obstacle_registry and self.obstacle_registry.obstacles
        ):
            self.voxel_map.generate_visualization()
        if self.obstacle_registry is not None:
            export_obstacle_map(self.obstacle_registry, self.config.obstacle_map_file)
            export_obstacle_map(self.obstacle_registry, self.config.shared_obstacle_map_file)

    def run(self) -> None:
        self.camera.open()
        width, height = self.camera.actual_resolution
        print(f"Camera resolution: {width} x {height}")
        print(f"Camera mode: {self.config.camera_mode}")
        print(f"Calibration: {self.config.calib_file}")
        print(f"Pose source: {self.config.pose_source.value}")
        print(f"Mines: enabled | Obstacles: {self.config.enable_obstacles and self.obstacle_detector is not None}")
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
                    left, right, _eye_w = stereo
                    mine_frame = left
                else:
                    left = right = None
                    mine_frame = frame

                mine_detections = self.detector.detect(mine_frame, timestamp=timestamp)
                fused_mines = self.process_mine_detections(mine_detections, timestamp)

                fused_obstacles = []
                if left is not None and right is not None:
                    fused_obstacles = self.process_obstacle_detections(left, right, timestamp)

                last_detect_ms = (time.perf_counter() - t0) * 1000.0

                for mine in fused_mines:
                    pos = mine.world_position
                    print(
                        f"Mine tag {mine.tag_id}: "
                        f"world=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                        f"conf={mine.confidence:.2f} obs={mine.observation_count}"
                    )
                for obstacle in fused_obstacles:
                    pos = obstacle.world_position
                    print(
                        f"Obstacle {obstacle.obstacle_id} ({obstacle.label}): "
                        f"world=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                        f"conf={obstacle.confidence:.2f} obs={obstacle.observation_count}"
                    )

                if self.config.enable_visualization:
                    display = self.detector.draw_detections(
                        mine_frame,
                        mine_detections,
                        world_positions=self._last_world_positions,
                    )
                    if self.obstacle_detector is not None:
                        display = self.obstacle_detector.draw_detections(
                            display, self._last_obstacle_detections
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
            if self.config.enable_visualization:
                cv2.destroyAllWindows()
            if self.csv_logger is not None:
                self.csv_logger.close()

        print(f"\nDiscovered mines: {len(self.mine_registry.mines)}")
        for tag_id, mine in sorted(self.mine_registry.mines.items()):
            pos = mine.world_position
            print(
                f"  tag {tag_id}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                f"observations={mine.observation_count}"
            )

        if self.obstacle_registry is not None:
            print(f"\nMapped obstacles: {len(self.obstacle_registry.obstacles)}")
            for obstacle_id, obstacle in sorted(self.obstacle_registry.obstacles.items()):
                pos = obstacle.world_position
                print(
                    f"  id {obstacle_id} ({obstacle.label}): "
                    f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                    f"observations={obstacle.observation_count}"
                )

        self._save_maps()
        print("Saved map visualization to occ_grid_proj.png")


# Backward-compatible alias
MineDetectionPipeline = PerceptionPipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IARC perception pipeline (mines + obstacles)")
    parser.add_argument("--config", type=Path, default=SLAM_DIR / "pipeline_config.json")
    parser.add_argument("--camera-index", type=int, default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--no-v4l2", action="store_true")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument("--pose-source", choices=["stub", "esp32"], default=None)
    parser.add_argument("--esp32-host", type=str, default=None)
    parser.add_argument("--calib", type=Path, default=None)
    parser.add_argument("--enable-obstacles", action="store_true")
    parser.add_argument("--stereo", action="store_true", help="Enable stereo camera mode + obstacles")
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
    if args.enable_obstacles or args.stereo:
        config.enable_obstacles = True
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
