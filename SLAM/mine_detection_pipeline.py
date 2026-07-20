"""
IARC mine detection pipeline.

Detects AprilTags on mines, transforms camera-frame detections into world
coordinates using the drone pose, fuses repeated observations per tag ID,
and updates the 2D mine map in SparseVoxelMap.
"""

from __future__ import annotations

import argparse
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
    StubPoseProvider,
    homogeneous_from_pose,
    load_calibration,
    tag_pose_to_world,
)
from sparse_voxel_map import SparseVoxelMap  # noqa: E402


class MineDetectionPipeline:
    def __init__(self, config: PipelineConfig | None = None):
        self.config = config or PipelineConfig()
        self.calibration = load_calibration(self.config.calib_file)
        self.detector = AprilTagDetector(
            calibration=self.calibration,
            tag_family=self.config.tag_family,
            tag_size_m=self.config.tag_size_m,
            min_confidence=self.config.min_confidence,
        )
        self.pose_provider = StubPoseProvider(
            position=self.config.stub_drone_position,
            quaternion_xyzw=self.config.stub_drone_quaternion,
        )
        self.drone_camera_transform = homogeneous_from_pose(
            self.config.drone_camera_position,
            self.config.drone_camera_quaternion,
        )
        self.mine_registry = MineRegistry(min_confidence=self.config.min_confidence)
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
        )
        self.csv_logger = (
            DetectionCsvLogger(self.config.csv_log_file)
            if self.config.enable_csv_log
            else None
        )

    def process_detections(self, detections, timestamp: float) -> list:
        world_drone = self.pose_provider.world_drone_transform(timestamp)
        drone_pose = self.pose_provider.get_pose(timestamp)
        self.voxel_map.update_drone_position(drone_pose.position)

        updated_mines = []
        for detection in detections:
            world_position, world_rotation = tag_pose_to_world(
                detection.translation_camera,
                detection.rotation_camera,
                world_drone,
                self.drone_camera_transform,
            )
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

        return updated_mines

    def run(self) -> None:
        self.camera.open()
        width, height = self.camera.actual_resolution
        print(f"Camera resolution: {width} x {height}")
        print(f"Calibration: {self.config.calib_file}")
        print(f"Field size: {self.config.field_x}m x {self.config.field_y}m")
        print("Press 'q' to quit")

        if self.calibration.image_size is not None:
            calib_w, calib_h = self.calibration.image_size
            if (calib_w, calib_h) != (width, height):
                print(
                    f"Warning: runtime resolution {width}x{height} differs from "
                    f"calibration resolution {calib_w}x{calib_h}"
                )

        frame_count = 0
        try:
            while True:
                ok, frame = self.camera.read()
                if not ok:
                    print("Failed to read camera frame")
                    break

                timestamp = time.time()
                detections = self.detector.detect(frame, timestamp=timestamp)
                fused_mines = self.process_detections(detections, timestamp)

                if fused_mines:
                    for mine in fused_mines:
                        pos = mine.world_position
                        print(
                            f"Mine tag {mine.tag_id}: "
                            f"world=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) "
                            f"conf={mine.confidence:.2f} "
                            f"obs={mine.observation_count}"
                        )

                if self.config.enable_visualization:
                    display = self.detector.draw_detections(frame, detections)
                    cv2.imshow("Mine Detection Pipeline", display)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                frame_count += 1
                if frame_count % 30 == 0 and self.mine_registry.mines:
                    self.voxel_map.generate_visualization()
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

        if self.mine_registry.mines:
            self.voxel_map.generate_visualization()
            print("Saved mine map visualization to occ_grid_proj.png")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IARC AprilTag mine detection pipeline")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--no-v4l2", action="store_true")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--no-csv", action="store_true")
    parser.add_argument(
        "--calib",
        type=Path,
        default=PipelineConfig().calib_file,
        help="Path to camera_calib.npz",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = PipelineConfig(
        camera_index=args.camera_index,
        request_width=args.width,
        request_height=args.height,
        use_v4l2=not args.no_v4l2,
        enable_visualization=args.visualize,
        enable_csv_log=not args.no_csv,
        calib_file=args.calib,
    )
    pipeline = MineDetectionPipeline(config)
    pipeline.run()


if __name__ == "__main__":
    main()
