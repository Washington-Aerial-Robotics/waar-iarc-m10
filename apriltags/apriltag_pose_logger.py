import cv2
import numpy as np
import csv
import time
import atexit
from pupil_apriltags import Detector

# ================= CONFIG =================
CAMERA_INDEX = 0
REQUEST_WIDTH = 1920
REQUEST_HEIGHT = 1080

CALIB_FILE = "camera_calib.npz"
TAG_FAMILY = "tag36h11"

# IMPORTANT: this must be the OUTER BLACK SQUARE width
TAG_SIZE_M = 0.0381   # 1.5 inches = 0.0381 m (ONLY if black square is 1.5")

LOG_FILE = "apriltag_log.csv"
# =========================================


# ---------- Logging setup ----------
log_file = open(LOG_FILE, "w", newline="")
log_writer = csv.writer(log_file)

log_writer.writerow([
    "timestamp",
    "tag_id",
    "x", "y", "z",
    "yaw", "pitch", "roll",
    "confidence"
])

atexit.register(log_file.close)
print(f"[LOG] Writing AprilTag pose data to {LOG_FILE}")


# ---------- Utility functions ----------
def clamp01(x):
    return max(0.0, min(1.0, x))


def polygon_area(pts):
    x = pts[:, 0]
    y = pts[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def rotmat_to_euler_zyx_degrees(R):
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


def tilt_angle_degrees(R):
    n = R @ np.array([0.0, 0.0, 1.0])
    n = n / (np.linalg.norm(n) + 1e-12)
    forward = np.array([0.0, 0.0, 1.0])
    cosang = float(np.dot(n, -forward))
    cosang = max(-1.0, min(1.0, cosang))
    return float(np.degrees(np.arccos(cosang)))


def confidence_score(decision_margin, area_px, tilt_deg):
    # Decision margin (if available)
    if decision_margin is None:
        dm_score = 0.5
    else:
        dm_score = clamp01((decision_margin - 20.0) / 60.0)

    # Area score (tuned for ~1080p + small tags)
    area_score = clamp01((area_px - 800.0) / 8000.0)

    # Tilt score (head-on best)
    tilt_score = clamp01((75.0 - tilt_deg) / 75.0)

    return clamp01(
        0.45 * dm_score +
        0.40 * area_score +
        0.15 * tilt_score
    )


# ---------- Main ----------
def main():
    # Load calibration
    calib = np.load(CALIB_FILE)
    K = calib["camera_matrix"]
    dist = calib["dist_coeffs"]

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    camera_params = (float(fx), float(fy), float(cx), float(cy))

    detector = Detector(
        families=TAG_FAMILY,
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.5,
        debug=False
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"Camera resolution: {actual_w} x {actual_h}")
    print(f"Using fx={fx:.1f}, fy={fy:.1f}")
    print("Press 'q' to quit")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        now = time.time()

        undist = cv2.undistort(frame, K, dist)
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=TAG_SIZE_M
        )

        for det in detections:
            tag_id = int(det.tag_id)

            pose_t = det.pose_t.reshape(3)
            R = det.pose_R

            yaw, pitch, roll = rotmat_to_euler_zyx_degrees(R)
            tilt = tilt_angle_degrees(R)

            area_px = polygon_area(det.corners)
            decision_margin = getattr(det, "decision_margin", None)

            conf = confidence_score(decision_margin, area_px, tilt)

            # ---- CSV LOGGING (THIS IS THE KEY PART) ----
            log_writer.writerow([
                now,
                tag_id,
                float(pose_t[0]),
                float(pose_t[1]),
                float(pose_t[2]),
                float(yaw),
                float(pitch),
                float(roll),
                float(conf)
            ])
            log_file.flush()
            # ------------------------------------------

            # Visual overlay (optional, for debugging)
            corners = det.corners.astype(int)
            cv2.polylines(undist, [corners], True, (0, 255, 0), 2)
            c = tuple(det.center.astype(int))

            cv2.putText(
                undist,
                f"ID {tag_id}  z={pose_t[2]:.2f}m  conf={conf:.2f}",
                (c[0] + 10, c[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        cv2.imshow("AprilTag Pose Logger", undist)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()