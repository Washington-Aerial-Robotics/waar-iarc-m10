import cv2
import numpy as np
import pandas as pd
from ultralytics import YOLO

from sparse_voxel_map import SparseVoxelMap

model = None  # initialized in main()

MAX_CAMERA_INDEX = 5
FORCED_CAMERA_INDEX = None  # set to an int (e.g. 2) from find_camera_index.py to skip auto-scan

FRAME_WIDTH = 2560
FRAME_HEIGHT = 720
EYE_WIDTH = 1280
HEIGHT = 720

FX = 700.0
FY = 700.0
CX = 640.0
CY = 360.0
BASELINE = 0.06

MIN_DEPTH_M = 0.2
MAX_DEPTH_M = 4.0
MIN_FRAGMENT_AREA = 150
MIN_OBJECT_AREA = 2000
MIN_VALID_DEPTH_PIXELS = 30
VIS_EVERY_N_FRAMES = 30
MERGE_GAP_PX = 30
CANNY_LOW = 50
CANNY_HIGH = 120
EDGE_DILATE_ITER = 2


def build_stereo_matcher():
    block_size = 7
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=144,  # must be a multiple of 16; covers down to ~0.29m
        blockSize=block_size,
        P1=8 * block_size ** 2,
        P2=32 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def compute_depth_map(left_gray, right_gray, stereo):
    disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
    valid_mask = disparity > 0.0

    depth_map = np.zeros_like(disparity)
    depth_map[valid_mask] = (FX * BASELINE) / disparity[valid_mask]

    valid_mask &= (depth_map >= MIN_DEPTH_M) & (depth_map <= MAX_DEPTH_M)
    return depth_map, valid_mask


def make_depth_display(depth_map, valid_mask):
    disp_vis = np.zeros(depth_map.shape, dtype=np.uint8)
    if np.any(valid_mask):
        vmin, vmax = np.percentile(depth_map[valid_mask], [2, 98])
        vmax = max(vmax, vmin + 1e-3)
        clipped = np.clip(depth_map, vmin, vmax)
        disp_vis = ((clipped - vmin) / (vmax - vmin) * 255).astype(np.uint8)

    disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
    disp_color[~valid_mask] = 0
    return disp_color


def _rects_within_gap(a, b, gap_px):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    return not (ax2 + gap_px < bx1 or bx2 + gap_px < ax1 or
                ay2 + gap_px < by1 or by2 + gap_px < ay1)


def merge_nearby_rects(rects, gap_px):
    boxes = [[x, y, x + w, y + h] for (x, y, w, h) in rects]

    changed = True
    while changed:
        changed = False
        merged = []
        used = [False] * len(boxes)
        for i in range(len(boxes)):
            if used[i]:
                continue
            current = boxes[i]
            used[i] = True
            for j in range(i + 1, len(boxes)):
                if used[j] or not _rects_within_gap(current, boxes[j], gap_px):
                    continue
                bx1, by1, bx2, by2 = boxes[j]
                current = [min(current[0], bx1), min(current[1], by1),
                           max(current[2], bx2), max(current[3], by2)]
                used[j] = True
                changed = True
            merged.append(current)
        boxes = merged

    return [(x1, y1, x2 - x1, y2 - y1) for (x1, y1, x2, y2) in boxes]


def detect_objects_yolo(left_bgr, model):
    results = model(left_bgr, verbose=False, conf=0.15)
    boxes = []
    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = float(box.conf[0])
            print(f"Detected: {label} ({conf:.2f})")
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            w = x2 - x1
            h = y2 - y1
            if w * h >= MIN_OBJECT_AREA:
                boxes.append((x1, y1, w, h, label))
    return boxes


def object_depth_range(depth_map, valid_mask, x, y, w, h):
    roi_depth = depth_map[y:y + h, x:x + w]
    roi_valid = valid_mask[y:y + h, x:x + w]
    values = roi_depth[roi_valid]
    if values.size < MIN_VALID_DEPTH_PIXELS:
        return None

    d_near, d_far = np.percentile(values, [5, 95])
    if d_far <= d_near:
        d_far = d_near + 0.01
    return float(d_near), float(d_far)


def backproject(u, v, d):
    x = (u - CX) * d / FX
    y = (v - CY) * d / FY
    z = d
    return np.array([x, y, z])


def get_3d_box_corners(x, y, w, h, d_near, d_far):
    pixel_corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
    corners = [backproject(u, v, d) for d in (d_near, d_far) for (u, v) in pixel_corners]
    return np.array(corners)


def draw_3d_box(image, x, y, w, h, label, d_near, d_far, color=(0, 255, 0)):
    # Back-projecting then reprojecting a pixel along the same camera ray always
    # lands on the same pixel, so the near/far faces coincide in image space.
    # The "back" face below is a shrunk inset used purely as a depth cue, not
    # a literal reprojection of the far-face 3D corners.
    front = np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]])
    center = np.array([x + w / 2.0, y + h / 2.0])
    back = ((front - center) * 0.85 + center).astype(int)
    front = front.astype(int)

    cv2.polylines(image, [front], isClosed=True, color=color, thickness=2)
    cv2.polylines(image, [back], isClosed=True, color=color, thickness=1)
    for (fx_, fy_), (bx_, by_) in zip(front, back):
        cv2.line(image, (fx_, fy_), (bx_, by_), color, 1)

    cv2.putText(image, f"{label} z:{d_near:.2f}-{d_far:.2f}m",
                (x, max(0, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def _eye_width_for(w, h):
    if w == 2560 and h == 720:
        return 1280
    if w == 1280 and h == 720:
        return 640
    return None


def _try_open(idx, forced):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        cap.release()
        return None

    if forced:
        # some USB stereo cameras only reach their full side-by-side
        # resolution in MJPG mode; request that explicitly as a fallback
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    ret, frame = cap.read()
    if not ret:
        cap.release()
        return None

    h, w = frame.shape[:2]
    eye_w = _eye_width_for(w, h)
    if eye_w is None:
        cap.release()
        return None

    mode = "forced MJPG" if forced else "default"
    print(f"Found stereo camera at index {idx} ({w}x{h}, {mode} mode)")
    return cap, eye_w


def find_stereo_camera(max_index=MAX_CAMERA_INDEX):
    # camera indices shift by OS/driver/whatever else is plugged in, so scan
    # for the first index that opens and reports a stereo-shaped frame
    # instead of assuming index 0 is always the stereo camera
    indices = [FORCED_CAMERA_INDEX] if FORCED_CAMERA_INDEX is not None else range(max_index)

    for idx in indices:
        result = _try_open(idx, forced=False) or _try_open(idx, forced=True)
        if result is not None:
            return result

        print(f"Camera at index {idx}: no stereo-shaped frame in default or forced mode, skipping")

    return None, None


def main():
    cap, eye_w = find_stereo_camera()
    if cap is None:
        print(f"Could not find a 2560x720 or 1280x720 camera on indices 0-{MAX_CAMERA_INDEX - 1}")
        return

    stereo = build_stereo_matcher()

    global model
    model = YOLO("yolov8n.pt")  # downloads automatically on first run

    origin_row = {"x": 0.0, "y": 0.0, "z": 0.0}
    voxel_map = SparseVoxelMap(pd.Series(origin_row), field_x=10, field_y=8)

    gps_position = np.array([0.0, 0.0, 1.5])
    quaternion = np.array([0.0, 0.0, 0.0, 1.0])

    frame_count = 0
    print("Press 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from camera")
            break

        left = frame[:, :eye_w]
        right = frame[:, eye_w:]

        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

        depth_map, valid_mask = compute_depth_map(left_gray, right_gray, stereo)

        boxes = detect_objects_yolo(left, model)

        display = left.copy()

        for (x, y, w, h, label) in boxes:
            depth_range = object_depth_range(depth_map, valid_mask, x, y, w, h)
            if depth_range is None:
                continue
            d_near, d_far = depth_range

            corners_3d = get_3d_box_corners(x, y, w, h, d_near, d_far)
            voxel_map.new_obstacle(corners_3d, quaternion=quaternion, gps_position=gps_position)

            draw_3d_box(display, x, y, w, h, label, d_near, d_far)

        cv2.imshow("Left Eye - 3D Detections", display)
        cv2.imshow("Depth Map", make_depth_display(depth_map, valid_mask))

        frame_count += 1
        if frame_count % VIS_EVERY_N_FRAMES == 0:
            voxel_map.generate_visualization()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # check a position we know has an obstacle
    print("\nOccupied cells in map:")
    found = False
    for x in np.arange(0, 10, 0.2):
        for y in np.arange(0, 8, 0.2):
            for z in np.arange(0, 3, 0.2):
                if voxel_map.is_obstacle(x, y, z):
                    print(f"  Obstacle at ({x:.1f}, {y:.1f}, {z:.1f})")
                    found = True
                    break  # only print first z hit per (x,y)
        
    if not found:
        print("  No obstacles found in range")

    print(f"\nTotal occupied voxels: {len(voxel_map.temporary_voxels)}")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
