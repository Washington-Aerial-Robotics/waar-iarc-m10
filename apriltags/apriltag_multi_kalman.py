import time
import cv2
import numpy as np
from pupil_apriltags import Detector


class TagTrack:
    """
    Constant-velocity Kalman filter for position (x,y,z) in meters.
    State: [x, y, z, vx, vy, vz]^T
    Measurement: [x, y, z]^T
    """
    def __init__(self, initial_xyz: np.ndarray, t0: float):
        self.kf = cv2.KalmanFilter(6, 3)

        # Measurement model: we observe position only
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=np.float32)

        # Process noise (tune: higher => more responsive, less smooth)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-3

        # Measurement noise (tune: higher => trust measurements less)
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 5e-3

        self.kf.errorCovPost = np.eye(6, dtype=np.float32) * 0.1

        # Initial state
        self.kf.statePost = np.array([
            [initial_xyz[0]],
            [initial_xyz[1]],
            [initial_xyz[2]],
            [0],
            [0],
            [0],
        ], dtype=np.float32)

        self.last_t = t0
        self.last_seen_t = t0
        self.misses = 0

    def _set_transition(self, dt: float):
        # Constant velocity model
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1],
        ], dtype=np.float32)

    def predict(self, t: float) -> np.ndarray:
        dt = max(1e-3, float(t - self.last_t))
        self._set_transition(dt)
        pred = self.kf.predict()
        self.last_t = t
        return pred[:3, 0].copy()

    def update(self, xyz: np.ndarray, t: float) -> np.ndarray:
        # Predict to this time, then correct with measurement
        _ = self.predict(t)
        meas = np.array([[xyz[0]], [xyz[1]], [xyz[2]]], dtype=np.float32)
        est = self.kf.correct(meas)
        self.last_seen_t = t
        self.misses = 0
        return est[:3, 0].copy()

    def mark_missed(self):
        self.misses += 1


def main(
    camera_index: int = 1,
    tag_family: str = "tag36h11",
    tag_size_m: float = 0.055,
    use_pose: bool = True,
    max_missed_frames: int = 15,
):
    detector = Detector(
        families=tag_family,
        nthreads=4,
        quad_decimate=1.0,   # raise to 1.5–2.0 for speed if needed
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        debug=False
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try camera_index=1 or 2.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    tracks: dict[int, TagTrack] = {}

    print("Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        t = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]

        # Camera intrinsics (replace with calibrated values for accuracy)
        fx = fy = float(w)
        cx, cy = w / 2.0, h / 2.0
        camera_params = (fx, fy, cx, cy)

        if use_pose:
            detections = detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=camera_params,
                tag_size=tag_size_m,
            )
        else:
            detections = detector.detect(gray)

        seen_ids = set()

        # --- Update tracks with measurements ---
        for det in detections:
            tag_id = int(det.tag_id)
            seen_ids.add(tag_id)

            corners = det.corners.astype(int)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

            cxy = det.center.astype(np.float32)

            # Measurement xyz:
            # - If pose enabled: use translation (meters)
            # - Else: use "pseudo xyz": x,y in pixels, z=0 (still smooths nicely)
            if use_pose and hasattr(det, "pose_t") and det.pose_t is not None:
                xyz = det.pose_t.reshape(3).astype(np.float32)  # meters
            else:
                xyz = np.array([cxy[0], cxy[1], 0.0], dtype=np.float32)

            if tag_id not in tracks:
                tracks[tag_id] = TagTrack(xyz, t)

            est_xyz = tracks[tag_id].update(xyz, t)

            # --- Draw smoothed info ---
            center_px = (int(cxy[0]), int(cxy[1]))
            cv2.circle(frame, center_px, 4, (0, 0, 255), -1)

            if use_pose and xyz[2] != 0.0:
                dist = float(np.linalg.norm(est_xyz))
                txt = f"ID {tag_id}  dist {dist:.2f}m"
            else:
                txt = f"ID {tag_id}"

            cv2.putText(
                frame, txt,
                (center_px[0] + 10, center_px[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2
            )

        # --- Predict tracks not seen this frame (optional visualization) ---
        dead_ids = []
        for tag_id, tr in tracks.items():
            if tag_id in seen_ids:
                continue

            tr.mark_missed()
            pred_xyz = tr.predict(t)

            # If not using pose, pred xyz is pixels
            if not use_pose:
                px = (int(pred_xyz[0]), int(pred_xyz[1]))
                cv2.circle(frame, px, 4, (255, 255, 0), -1)
                cv2.putText(
                    frame, f"ID {tag_id} (pred)",
                    (px[0] + 10, px[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2
                )

            if tr.misses > max_missed_frames:
                dead_ids.append(tag_id)

        for tag_id in dead_ids:
            del tracks[tag_id]

        cv2.imshow("AprilTag Multi + Kalman", frame)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(
        camera_index=1,
        tag_family="tag36h11",
        tag_size_m=0.055,   # set your printed tag size
        use_pose=True,      # set False if you only want 2D smoothing
        max_missed_frames=15
    )