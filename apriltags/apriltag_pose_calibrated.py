import cv2
import numpy as np
from pupil_apriltags import Detector

CALIB_FILE = "camera_calib.npz"

def main(camera_index=0, tag_size_m=0.055, family="tag36h11"):
    calib = np.load(CALIB_FILE)
    K = calib["camera_matrix"]
    dist = calib["dist_coeffs"]

    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    camera_params = (float(fx), float(fy), float(cx), float(cy))

    detector = Detector(
        families=family,
        nthreads=4,
        quad_decimate=1.0,
        refine_edges=True
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try camera_index=1 or 2.")

    print("Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # Undistort improves pose quality a lot
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
            dist_m = float(np.linalg.norm(t))

            cv2.putText(
                undist,
                f"ID {det.tag_id}  dist {dist_m:.2f}m",
                (c[0] + 10, c[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        cv2.imshow("AprilTag Pose (Calibrated)", undist)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(camera_index=0, tag_size_m=0.055, family="tag36h11")