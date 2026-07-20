import cv2
import numpy as np
from pupil_apriltags import Detector

def wrap_deg(a):
    a = (a + 180) % 360 - 180
    return a

def main(camera_index=1, family="tag36h11"):
    detector = Detector(
        families=family,
        nthreads=4,
        quad_decimate=1.0,
        refine_edges=True
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    print("Press 'q' to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        for det in detections:
            corners = det.corners  # float, order: TL, TR, BR, BL
            c = det.center

            # Vector from TL -> TR gives the tag's "top edge" direction in image
            v = corners[1] - corners[0]
            angle_rad = np.arctan2(v[1], v[0])   # y, x
            angle_deg = wrap_deg(np.degrees(angle_rad))

            # Draw outline
            pts = corners.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            cx, cy = int(c[0]), int(c[1])
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            # Draw top-edge direction arrow
            tip = (int(cx + 60 * np.cos(angle_rad)), int(cy + 60 * np.sin(angle_rad)))
            cv2.arrowedLine(frame, (cx, cy), tip, (255, 255, 0), 2, tipLength=0.2)

            cv2.putText(
                frame,
                f"ID {det.tag_id}  img-rot {angle_deg:+.1f} deg",
                (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        cv2.imshow("AprilTag Angle (No Calibration)", frame)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(camera_index=1, family="tag36h11")