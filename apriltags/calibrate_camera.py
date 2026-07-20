import cv2
import numpy as np
import time

# === SETTINGS YOU MAY CHANGE ===
CAMERA_INDEX = 0              # change if needed
REQUEST_WIDTH = 1920          # try 1920x1080 for small tags
REQUEST_HEIGHT = 1080
CHESSBOARD_SIZE = (9, 6)      # inner corners (columns, rows)
SQUARE_SIZE_M = 0.022225       # size of ONE chessboard square side, in meters
OUTPUT_FILE = "camera_calib.npz"


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera. Try a different CAMERA_INDEX.")

    # Request resolution (drivers may ignore)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_HEIGHT)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Requested resolution: {REQUEST_WIDTH}x{REQUEST_HEIGHT}")
    print(f"Actual resolution:    {actual_w}x{actual_h}")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    # Prepare object points: (0,0,0), (1,0,0), ... scaled by square size
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M

    objpoints = []  # 3D points in world coordinates
    imgpoints = []  # 2D points in image pixels

    last_capture_t = 0.0

    print("\nControls:")
    print("  SPACE = capture (only if corners found)")
    print("  C     = calibrate & save (need ~20-40 captures for best results)")
    print("  Q     = quit\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Failed to read frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, flags)

        vis = frame.copy()
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(vis, CHESSBOARD_SIZE, corners2, found)
            cv2.putText(vis, "Corners found", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            corners2 = None
            cv2.putText(vis, "No corners", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.putText(vis, f"Captures: {len(objpoints)}", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        cv2.imshow("Camera Calibration", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        # Capture
        if key == 32:  # SPACE
            now = time.time()
            if now - last_capture_t < 0.25:
                continue
            last_capture_t = now

            if not found:
                print("Capture skipped: corners not found.")
                continue

            objpoints.append(objp.copy())
            imgpoints.append(corners2)
            print(f"Captured #{len(objpoints)}")

        # Calibrate
        if key == ord("c"):
            if len(objpoints) < 10:
                print("Need at least ~10 captures (20-40 recommended).")
                continue

            image_size = (gray.shape[1], gray.shape[0])  # (width, height)
            print("Calibrating...")

            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, image_size, None, None
            )

            if not ret:
                print("Calibration failed.")
                continue

            # RMSE reprojection error (pixels)
            total_sq_err = 0.0
            total_pts = 0

            for i in range(len(objpoints)):
                projected, _ = cv2.projectPoints(
                    objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
                )
                err = cv2.norm(imgpoints[i], projected, cv2.NORM_L2)
                total_sq_err += err * err
                total_pts += len(projected)

            rmse = float(np.sqrt(total_sq_err / total_pts))

            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]

            print("\n=== Calibration results ===")
            print("Camera matrix (K):\n", camera_matrix)
            print("Distortion coeffs:\n", dist_coeffs.ravel())
            print(f"RMSE reprojection error (px): {rmse:.4f}")
            print(f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")

            np.savez(
                OUTPUT_FILE,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                image_size=np.array(image_size),
                rmse=np.array([rmse]),
                chessboard_size=np.array(CHESSBOARD_SIZE),
                square_size_m=np.array([SQUARE_SIZE_M]),
            )
            print(f"Saved calibration to: {OUTPUT_FILE}\n")
            print("Tip: keep using the same resolution you calibrated with.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()