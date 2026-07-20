import cv2
import numpy as np
from pupil_apriltags import Detector

CALIB_FILE = "camera_calib.npz"

# If confidence < this, we won't display it (and you could also skip sending it to a filter)
CONF_MIN = 0.1


def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x


def polygon_area(pts: np.ndarray) -> float:
    """
    Shoelace formula for polygon area. pts shape: (4,2)
    Returns area in pixel^2.
    """
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def rotmat_to_euler_zyx_degrees(R: np.ndarray):
    r20 = float(R[2, 0])
    r20 = max(-1.0, min(1.0, r20))

    pitch = np.arcsin(-r20)

    if abs(r20) > 0.9999:
        yaw = np.arctan2(-R[0, 1], R[1, 1])
        roll = 0.0
    else:
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])

    return np.degrees([yaw, pitch, roll])


def tilt_angle_degrees(R: np.ndarray):
    """
    0° = tag faces the camera head-on, 90° = edge-on.
    """
    n = R @ np.array([0.0, 0.0, 1.0])
    n = n / (np.linalg.norm(n) + 1e-12)

    forward = np.array([0.0, 0.0, 1.0])
    cosang = float(np.dot(n, -forward))
    cosang = max(-1.0, min(1.0, cosang))
    return float(np.degrees(np.arccos(cosang)))


def confidence_from_metrics(decision_margin, area_px, reproj_rmse_px, tilt_deg):
    """
    Blend multiple signals into a confidence value in [0,1].

    Tune constants if needed:
    - Default capture is 4K; area thresholds below may need scaling vs 1080p for small tags.
    - If you're on 640x480, lower the area thresholds.
    """
    # decision_margin: higher is better. If missing, treat as neutral.
    if decision_margin is None:
        dm_score = 0.5
    else:
        # 20 -> 0, 80 -> 1 (clamped)
        dm_score = clamp01((float(decision_margin) - 20.0) / 60.0)

    # Area score: bigger in pixels => better
    # 800 -> 0, 8800 -> 1 (clamped)  (tune these to your resolution/tag size)
    area_score = clamp01((float(area_px) - 800.0) / 8000.0)

    # Reprojection RMSE: lower is better
    # 0.5px great, 3px meh, 6px bad
    reproj_score = clamp01((6.0 - float(reproj_rmse_px)) / 5.5)

    # Tilt: head-on is best
    # 0deg -> 1, 75deg -> 0
    tilt_score = clamp01((75.0 - float(tilt_deg)) / 75.0)

    # Weighted blend
    conf = (
        0.40 * dm_score +
        0.25 * area_score +
        0.25 * reproj_score +
        0.10 * tilt_score
    )
    return clamp01(conf)


def pose_reprojection_rmse_px(det_corners_px: np.ndarray, R: np.ndarray, t: np.ndarray,
                              tag_size_m: float, K: np.ndarray, dist: np.ndarray) -> float:
    """
    Compute RMSE (in pixels) between detected corners and reprojected corners from estimated pose.
    """
    s = tag_size_m / 2.0
    # Tag corners in tag frame (meters). Ordering should match det.corners (TL,TR,BR,BL) approximately.
    tag_obj = np.array([
        [-s, -s, 0.0],
        [ s, -s, 0.0],
        [ s,  s, 0.0],
        [-s,  s, 0.0],
    ], dtype=np.float32)

    rvec, _ = cv2.Rodrigues(R.astype(np.float64))
    tvec = t.reshape(3, 1).astype(np.float64)

    proj, _ = cv2.projectPoints(tag_obj, rvec, tvec, K.astype(np.float64), dist.astype(np.float64))
    proj = proj.reshape(-1, 2).astype(np.float64)

    det_corners_px = det_corners_px.astype(np.float64)

    # RMSE over 4 corners
    return float(np.sqrt(np.mean(np.sum((proj - det_corners_px) ** 2, axis=1))))


def main(
    camera_index=1,
    tag_size_m=0.0254,
    family="tag36h11",
    request_width=3840,
    request_height=2160,
):
    # Load calibration
    calib = np.load(CALIB_FILE)
    K = calib["camera_matrix"]
    dist = calib["dist_coeffs"]

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    camera_params = (float(fx), float(fy), float(cx), float(cy))

    detector = Detector(
        families=family,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.5,
        debug=False
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    # Request high resolution for small tags
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, request_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, request_height)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {actual_w} x {actual_h}")
    print(f"Using fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
    print(f"Tag size (m): {tag_size_m} | family: {family}")
    print(f"CONF_MIN: {CONF_MIN}")
    print("Note: tag_size_m should be the OUTER BLACK SQUARE width, not paper size.")
    print("Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # Undistort for better pose
        undist = cv2.undistort(frame, K, dist)
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size_m
        )

        for det in detections:
            # Basic geometry + pose
            corners_f = det.corners  # float corners
            corners_i = corners_f.astype(int)
            c = tuple(det.center.astype(int))

            t = det.pose_t.reshape(3)
            R = det.pose_R

            dist_m = float(np.linalg.norm(t))
            tilt = tilt_angle_degrees(R)
            yaw, pitch, roll = rotmat_to_euler_zyx_degrees(R)

            # Confidence metrics
            area_px = polygon_area(corners_f)
            decision_margin = getattr(det, "decision_margin", None)

            rmse_px = pose_reprojection_rmse_px(
                det_corners_px=corners_f,
                R=R,
                t=t,
                tag_size_m=tag_size_m,
                K=K,
                dist=dist
            )

            conf = confidence_from_metrics(decision_margin, area_px, rmse_px, tilt)

            if conf < CONF_MIN:
                # Skip low-confidence detections
                continue

            # Draw
            cv2.polylines(undist, [corners_i], True, (0, 255, 0), 2)
            cv2.circle(undist, c, 4, (0, 0, 255), -1)

            cv2.putText(
                undist,
                f"ID {det.tag_id}  dist {dist_m:.2f}m  conf {conf:.2f}",
                (c[0] + 10, c[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.putText(
                undist,
                f"tilt {tilt:.1f} deg  rmse {rmse_px:.2f}px  area {area_px:.0f}px",
                (c[0] + 10, c[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

            cv2.putText(
                undist,
                f"yaw {yaw:+.1f}  pitch {pitch:+.1f}  roll {roll:+.1f}",
                (c[0] + 10, c[1] + 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

            if decision_margin is not None:
                cv2.putText(
                    undist,
                    f"decision_margin {float(decision_margin):.1f}",
                    (c[0] + 10, c[1] + 66),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2
                )

        cv2.imshow("AprilTag Pose (Calibrated + Angles + Confidence)", undist)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(
        camera_index=1,
        tag_size_m=0.0254,
        family="tag36h11",
        request_width=3840,
        request_height=2160
    )