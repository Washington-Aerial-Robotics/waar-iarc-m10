"""Detect AprilTag mine markers and publish map-frame MAS beliefs.

The node consumes the rectified left camera image and its CameraInfo.  OpenCV
returns tag translation in the conventional optical frame (x right, y down,
z forward); the SLAM URDF supplies the transform from that optical frame into
``map``.  Detections are always candidates.  A separate close-range
verification event is published for the physical executor to turn into a
TaskResult only when it is actually servicing the matching task.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from mas_interfaces.msg import MineBelief
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import MineTrack, detection_confidence, inside_arena, transform_point


def _polygon_area(corners: np.ndarray) -> float:
    points = corners.reshape(4, 2)
    x = points[:, 0]
    y = points[:, 1]
    return abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))) * 0.5


class AprilTagMineNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_mine_node")
        self.declare_parameter("drone_id", "d1")
        self.declare_parameter("image_topic", "/camera/left/image_rect")
        self.declare_parameter("camera_info_topic", "/camera/left/camera_info")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("tag_size_m", 0.0381)
        self.declare_parameter("minimum_pixel_area", 225.0)
        self.declare_parameter("reprojection_error_scale_px", 4.0)
        self.declare_parameter("minimum_confidence", 0.25)
        self.declare_parameter("verification_confidence", 0.65)
        self.declare_parameter("verification_max_distance_m", 2.0)
        self.declare_parameter("max_detection_distance_m", 15.0)
        self.declare_parameter("republish_interval_s", 2.0)
        self.declare_parameter("arena_x_min", 0.0)
        self.declare_parameter("arena_x_max", 91.44)
        self.declare_parameter("arena_y_min", 0.0)
        self.declare_parameter("arena_y_max", 24.38)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.minimum_pixel_area = float(self.get_parameter("minimum_pixel_area").value)
        self.error_scale_px = float(
            self.get_parameter("reprojection_error_scale_px").value
        )
        self.minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self.verification_confidence = float(
            self.get_parameter("verification_confidence").value
        )
        self.verification_max_distance_m = float(
            self.get_parameter("verification_max_distance_m").value
        )
        self.max_detection_distance_m = float(
            self.get_parameter("max_detection_distance_m").value
        )
        self.republish_interval_s = float(
            self.get_parameter("republish_interval_s").value
        )
        self.arena_bounds = (
            float(self.get_parameter("arena_x_min").value),
            float(self.get_parameter("arena_x_max").value),
            float(self.get_parameter("arena_y_min").value),
            float(self.get_parameter("arena_y_max").value),
        )
        if self.tag_size_m <= 0.0:
            raise ValueError("tag_size_m must be measured and positive")

        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
            raise RuntimeError(
                "OpenCV was built without aruco/AprilTag support; install "
                "python3-opencv with contrib modules"
            )
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        parameters = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, "DetectorParameters")
            else cv2.aruco.DetectorParameters_create()
        )
        self._detector = (
            cv2.aruco.ArucoDetector(dictionary, parameters)
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )
        self._dictionary = dictionary
        self._parameters = parameters
        self._bridge = CvBridge()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._camera_matrix: Optional[np.ndarray] = None
        self._distortion: Optional[np.ndarray] = None
        self._tracks: Dict[str, MineTrack] = {}
        self._last_publish: Dict[str, float] = {}

        self._belief_pub = self.create_publisher(
            MineBelief, f"/{self.drone_id}/mine_candidates", 10
        )
        self._verification_pub = self.create_publisher(
            String, f"/{self.drone_id}/verification_result", 10
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            5,
        )
        self.get_logger().info(
            f"AprilTag mine perception ready for {self.drone_id}; "
            f"tag_size={self.tag_size_m:.4f}m"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        # This node consumes image_rect, so use the rectified projection
        # matrix and no distortion. CameraInfo.K/D describe the raw image.
        projection = np.asarray(msg.p, dtype=np.float64).reshape(3, 4)
        matrix = projection[:, :3]
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.get_logger().error("Ignoring invalid rectified CameraInfo focal length")
            return
        self._camera_matrix = matrix
        self._distortion = np.zeros(5, dtype=np.float64)

    def _detect(self, gray: np.ndarray):
        if self._detector is not None:
            return self._detector.detectMarkers(gray)[:2]
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._dictionary, parameters=self._parameters
        )
        return corners, ids

    def _on_image(self, msg: Image) -> None:
        if self._camera_matrix is None or self._distortion is None:
            return
        try:
            gray = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            corners, ids = self._detect(gray)
        except Exception as exc:  # cv_bridge/OpenCV errors must not kill flight graph
            self.get_logger().error(f"AprilTag image processing failed: {exc}")
            return
        if ids is None:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Skipping detections without {self.map_frame} <- "
                f"{msg.header.frame_id} TF: {exc}"
            )
            return

        half = self.tag_size_m * 0.5
        object_points = np.asarray(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        tf_translation = (translation.x, translation.y, translation.z)
        tf_quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)

        for marker_corners, tag_id_value in zip(corners, ids.flatten()):
            image_points = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self._camera_matrix,
                self._distortion,
                flags=getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE),
            )
            if not success:
                continue
            camera_point = tuple(float(value) for value in tvec.reshape(3))
            distance = math.sqrt(sum(value * value for value in camera_point))
            if not 0.0 < distance <= self.max_detection_distance_m:
                continue
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec, self._camera_matrix, self._distortion
            )
            residual = projected.reshape(4, 2) - image_points
            rmse = float(np.sqrt(np.mean(np.square(residual))))
            confidence = detection_confidence(
                rmse,
                _polygon_area(image_points),
                self.minimum_pixel_area,
                self.error_scale_px,
            )
            if confidence < self.minimum_confidence:
                continue
            try:
                map_point = transform_point(camera_point, tf_translation, tf_quaternion)
            except ValueError as exc:
                self.get_logger().error(f"Invalid perception transform: {exc}")
                continue
            self._record_detection(
                int(tag_id_value), map_point[0], map_point[1], confidence, distance
            )

    def _record_detection(
        self, tag_id: int, x: float, y: float, confidence: float, distance: float
    ) -> None:
        if not inside_arena(x, y, self.arena_bounds):
            self.get_logger().warning(
                f"Rejecting tag {tag_id} outside arena at ({x:.2f}, {y:.2f})"
            )
            return
        mine_id = f"tag_{tag_id}"
        track = self._tracks.get(mine_id)
        changed = True
        if track is None:
            track = MineTrack(mine_id, x, y, confidence)
            self._tracks[mine_id] = track
        else:
            changed = track.update(x, y, confidence)
        now = time.monotonic()
        due = now - self._last_publish.get(mine_id, 0.0) >= self.republish_interval_s
        if changed or due:
            belief = MineBelief()
            belief.mine_id = mine_id
            belief.x = track.x
            belief.y = track.y
            belief.confidence = float(track.confidence)
            belief.status = "candidate"
            belief.last_updated_by = self.drone_id
            belief.seq = track.sequence
            belief.stamp = self.get_clock().now().to_msg()
            self._belief_pub.publish(belief)
            self._last_publish[mine_id] = now

        if (
            track.verification_ready
            and confidence >= self.verification_confidence
            and distance <= self.verification_max_distance_m
        ):
            event = String()
            event.data = json.dumps(
                {
                    "mine_id": mine_id,
                    "outcome": "confirmed",
                    "confidence": confidence,
                    "x": track.x,
                    "y": track.y,
                    "distance_m": distance,
                },
                separators=(",", ":"),
            )
            self._verification_pub.publish(event)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagMineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
