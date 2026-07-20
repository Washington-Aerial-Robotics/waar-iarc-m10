import cv2
import numpy as np
from pupil_apriltags import Detector

def main(camera_index: int = 0, tag_size_m: float = 0.055):
    detector = Detector(
        families="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        refine_edges=True
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try camera_index=1 or 2.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]

        # Rough intrinsics (replace with calibrated values for accuracy)
        fx = fy = float(w)
        cx, cy = w / 2.0, h / 2.0
        camera_params = (fx, fy, cx, cy)

        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size_m
        )

        for det in detections:
            corners = det.corners.astype(int)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

            c = tuple(det.center.astype(int))
            cv2.circle(frame, c, 4, (0, 0, 255), -1)

            # det.pose_t is translation (meters), det.pose_R is rotation matrix
            t = det.pose_t.reshape(3)
            dist = np.linalg.norm(t)

            cv2.putText(frame, f"ID {det.tag_id}  dist {dist:.2f}m",
                        (c[0] + 10, c[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("AprilTag Webcam (Pose)", frame)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(camera_index=0, tag_size_m=0.055)