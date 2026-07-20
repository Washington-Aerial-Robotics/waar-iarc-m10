import cv2
import numpy as np
from pupil_apriltags import Detector

CALIB_FILE = "camera_calib.npz"


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
    n = R @ np.array([0.0, 0.0, 1.0])
    n = n / (np.linalg.norm(n) + 1e-12)

    forward = np.array([0.0, 0.0, 1.0])
    cosang = float(np.dot(n, -forward))
    cosang = max(-1.0, min(1.0, cosang))
    return float(np.degrees(np.arccos(cosang)))


def main(
    camera_index=0,
    tag_size_m=0.0254,
    family="tag36h11",
    request_width=1920,
    request_height=1080
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
        quad_decimate=1.0,      # keep 1.0 for small tags
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.5,  # try 0.25–1.0 if needed
        debug=False
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    # Request higher resolution (small tags benefit a lot)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, request_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, request_height)

    # If the camera ignored it, try a common fallback (720p)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if actual_w != request_width or actual_h != request_height:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print("Press 'q' to quit.")
    print(f"Camera resolution: {actual_w} x {actual_h}")
    print(f"Using fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
    print(f"Tag size (m): {tag_size_m}  | family: {family}")
    print("Note: tag_size_m should be the width of the OUTER BLACK SQUARE, not the paper.")

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
            corners = det.corners.astype(int)
            cv2.polylines(undist, [corners], True, (0, 255, 0), 2)

            c = tuple(det.center.astype(int))
            cv2.circle(undist, c, 4, (0, 0, 255), -1)

            t = det.pose_t.reshape(3)
            R = det.pose_R

            dist_m = float(np.linalg.norm(t))
            tilt = tilt_angle_degrees(R)
            yaw, pitch, roll = rotmat_to_euler_zyx_degrees(R)

            cv2.putText(
                undist,
                f"ID {det.tag_id}  dist {dist_m:.2f}m",
                (c[0] + 10, c[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.putText(
                undist,
                f"tilt {tilt:.1f} deg",
                (c[0] + 10, c[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            cv2.putText(
                undist,
                f"yaw {yaw:+.1f}  pitch {pitch:+.1f}  roll {roll:+.1f}",
                (c[0] + 10, c[1] + 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        cv2.imshow("AprilTag Pose (Calibrated + Angles)", undist)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(
        camera_index=0,
        tag_size_m=0.0254,
        family="tag36h11",
        request_width=1920,
        request_height=1080
    )