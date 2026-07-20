import cv2
import numpy as np
from pupil_apriltags import Detector

try:
    import mediapipe as mp
except ImportError as exc:
    raise ImportError(
        "mediapipe is required for human tracking. Install with: pip install mediapipe"
    ) from exc

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


CALIB_FILE = "camera_calib.npz"


def get_mediapipe_solutions():
    # MediaPipe packaging differs across some Windows/Python builds.
    if hasattr(mp, "solutions"):
        return mp.solutions
    try:
        from mediapipe.python import solutions as mp_solutions

        return mp_solutions
    except Exception:
        return None


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


def draw_apriltags(frame: np.ndarray, detections):
    for det in detections:
        corners = det.corners.astype(int)
        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
        center = tuple(det.center.astype(int))
        cv2.circle(frame, center, 4, (0, 255, 255), -1)

        t = det.pose_t.reshape(3)
        dist_m = float(np.linalg.norm(t))
        tilt = tilt_angle_degrees(det.pose_R)
        yaw, pitch, roll = rotmat_to_euler_zyx_degrees(det.pose_R)

        cv2.putText(
            frame,
            f"Tag {det.tag_id}  {dist_m:.2f}m",
            (center[0] + 10, center[1] - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"tilt {tilt:.1f}  yaw {yaw:+.1f} pitch {pitch:+.1f} roll {roll:+.1f}",
            (center[0] + 10, center[1] + 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )


def wrist_above_shoulder(landmarks, side: str) -> bool:
    # BlazePose landmark indices:
    # 11 left shoulder, 12 right shoulder, 15 left wrist, 16 right wrist.
    if side == "left":
        wrist = landmarks[15]
        shoulder = landmarks[11]
    else:
        wrist = landmarks[16]
        shoulder = landmarks[12]
    return wrist.y < shoulder.y


def draw_human_pose(frame: np.ndarray, pose_result, mp_pose, mp_drawing):
    if not pose_result.pose_landmarks:
        return "No person"

    mp_drawing.draw_landmarks(
        frame,
        pose_result.pose_landmarks,
        mp_pose.POSE_CONNECTIONS,
        mp_drawing.DrawingSpec(color=(255, 128, 0), thickness=2, circle_radius=2),
        mp_drawing.DrawingSpec(color=(0, 180, 255), thickness=2, circle_radius=2),
    )

    landmarks = pose_result.pose_landmarks.landmark
    left_up = wrist_above_shoulder(landmarks, "left")
    right_up = wrist_above_shoulder(landmarks, "right")

    if left_up and right_up:
        return "Both hands up!"
    if left_up or right_up:
        return "One hand up!"
    return "Pose tracked"


def draw_mediapipe_hands(frame: np.ndarray, hands_result, mp_hands, mp_drawing):
    if not hands_result.multi_hand_landmarks:
        return 0

    count = 0
    for hand_landmarks in hands_result.multi_hand_landmarks:
        mp_drawing.draw_landmarks(
            frame,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(255, 0, 255), thickness=2, circle_radius=2),
            mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=2, circle_radius=2),
        )
        count += 1
    return count


def draw_face_boxes(frame: np.ndarray, face_detector, gray_frame: np.ndarray):
    faces = face_detector.detectMultiScale(
        gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60)
    )
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (120, 255, 120), 2)
        cv2.putText(
            frame,
            "Face",
            (x, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (120, 255, 120),
            2,
        )
    return len(faces)


def draw_human_pose_yolo(frame: np.ndarray, yolo_results):
    if not yolo_results:
        return "No person"

    # COCO skeleton pairs for YOLOv8 keypoints (17 points).
    edges = [
        (5, 7), (7, 9),    # left arm
        (6, 8), (8, 10),   # right arm
        (5, 6),            # shoulders
        (5, 11), (6, 12),  # torso
        (11, 12),          # hips
        (11, 13), (13, 15),# left leg
        (12, 14), (14, 16) # right leg
    ]

    person_found = False
    person_count = 0
    left_up = False
    right_up = False

    for result in yolo_results:
        if result.boxes is not None:
            boxes_xyxy = result.boxes.xyxy.cpu().numpy()
            boxes_cls = result.boxes.cls.cpu().numpy()
            boxes_conf = result.boxes.conf.cpu().numpy()
            for i in range(len(boxes_xyxy)):
                if int(boxes_cls[i]) != 0:
                    continue
                person_count += 1
                x1, y1, x2, y2 = np.int32(boxes_xyxy[i])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 220, 255), 2)
                cv2.putText(
                    frame,
                    f"Human detected ({boxes_conf[i]:.2f})",
                    (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (40, 220, 255),
                    2,
                )

        if result.keypoints is None:
            continue
        xy = result.keypoints.xy.cpu().numpy()      # [N,17,2]
        conf = result.keypoints.conf.cpu().numpy()  # [N,17]
        if len(xy) == 0:
            continue

        person_found = True
        pts = xy[0]
        cfs = conf[0]

        for a, b in edges:
            if cfs[a] > 0.35 and cfs[b] > 0.35:
                pa = tuple(np.int32(pts[a]))
                pb = tuple(np.int32(pts[b]))
                cv2.line(frame, pa, pb, (0, 180, 255), 2)

        for i in range(17):
            if cfs[i] > 0.35:
                p = tuple(np.int32(pts[i]))
                cv2.circle(frame, p, 3, (255, 128, 0), -1)

        # wrist y < shoulder y means hand raised (image coordinates).
        if cfs[9] > 0.35 and cfs[5] > 0.35:
            left_up = pts[9][1] < pts[5][1]
        if cfs[10] > 0.35 and cfs[6] > 0.35:
            right_up = pts[10][1] < pts[6][1]
        break

    if not person_found:
        return "No person", person_count
    if left_up and right_up:
        return "Both hands up!", person_count
    if left_up or right_up:
        return "One hand up!", person_count
    return "Pose tracked", person_count


def main(
    camera_index=0,
    tag_size_m=0.0254,
    family="tag36h11",
    request_width=1920,
    request_height=1080,
):
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
        debug=False,
    )

    mp_solutions = get_mediapipe_solutions()
    use_mediapipe = mp_solutions is not None
    use_yolo = False

    if use_mediapipe:
        mp_pose = mp_solutions.pose
        mp_hands = mp_solutions.hands
        mp_drawing = mp_solutions.drawing_utils
        pose = mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        print("Human tracking backend: MediaPipe Solutions")
    else:
        if YOLO is None:
            raise RuntimeError(
                "No compatible human tracking backend found. "
                "Install ultralytics: pip install ultralytics"
            )
        yolo_model = YOLO("yolov8n-pose.pt")
        use_yolo = True
        print("Human tracking backend: YOLOv8 Pose (MediaPipe Solutions unavailable)")

    face_detector = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, request_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, request_height)

    print("Demo controls: press 'q' to quit")
    print("Try this with students: raise both hands + move an AprilTag in view.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        undist = cv2.undistort(frame, K, dist)
        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size_m,
        )
        draw_apriltags(undist, detections)

        hand_count = 0
        person_count = 0
        face_count = draw_face_boxes(undist, face_detector, gray)

        if use_mediapipe:
            rgb = cv2.cvtColor(undist, cv2.COLOR_BGR2RGB)
            pose_result = pose.process(rgb)
            pose_msg = draw_human_pose(undist, pose_result, mp_pose, mp_drawing)
            hands_result = hands.process(rgb)
            hand_count = draw_mediapipe_hands(undist, hands_result, mp_hands, mp_drawing)
            person_count = 1 if pose_result.pose_landmarks else 0
        elif use_yolo:
            yolo_results = yolo_model(undist, verbose=False)
            pose_msg, person_count = draw_human_pose_yolo(undist, yolo_results)
            # In YOLO fallback mode, wrists act as a simple "hand tracking" cue.
            hand_count = person_count * 2
        else:
            pose_msg = "No backend"

        cv2.rectangle(undist, (10, 10), (780, 128), (0, 0, 0), -1)
        cv2.putText(
            undist,
            f"AprilTags: {len(detections)}",
            (20, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            undist,
            f"Human tracking: {pose_msg}",
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 200, 255),
            2,
        )
        cv2.putText(
            undist,
            f"Humans: {person_count}  Hands: {hand_count}  Faces: {face_count}",
            (20, 98),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.imshow("AprilTag + Human Tracking Demo", undist)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    if use_mediapipe:
        pose.close()
        hands.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
