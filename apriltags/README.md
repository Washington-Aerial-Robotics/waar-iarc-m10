# AprilTag Detection + Pose Toolkit

Detect AprilTags from webcam video, estimate tag pose, and log telemetry. Includes both local Python pipelines and a hostable browser detector.

## Quick Snapshot
- **What is this?** A practical AprilTag detection system with Python CV scripts and a WebAssembly web app.
- **Who is it for?** Robotics developers, computer-vision builders, and teams that need fast fiducial tracking in the lab or in the field.
- **Why did I build it?** I wanted reliable pose estimation plus a no-install browser detector that anyone can open and test.
- **Can I run it?** Yes. Run locally with Python (`venv`) or run the web app with a static server.
- **Is there a demo?** Yes: [https://apriltag-navy.vercel.app](https://apriltag-navy.vercel.app)
- **What did I specifically implement?**
  - Python webcam detection and pose estimation flows
  - Pose/confidence scoring utilities
  - CSV pose logger pipeline
  - Browser detector UI (camera, overlays, tag list)
  - Vercel-ready static hosting config for WASM assets

## What I Built (My Contribution)
- Built and integrated detection scripts using `pupil_apriltags` + OpenCV.
- Implemented confidence scoring from decision margin, area, reprojection error, and tilt.
- Added pose logging to `apriltag_log.csv` with timestamped measurements.
- Implemented the browser detector in `web/` using a worker + WASM AprilTag runtime.
- Configured static deployment behavior and WASM content-type handling for hosting.

## Repo Structure
- `apriltag_pose_logger.py` - webcam pose estimation + CSV logging.
- `apriltag_confidence.py` - detection + confidence scoring.
- `apriltag_pose_calibrated.py` / `apriltag_pose_calibrated_angles.py` - calibrated pose workflows.
- `apriltag_multi_kalman.py` - multi-tag tracking/filtering experiments.
- `calibrate_camera.py` - camera calibration utility.
- `web/` - browser detector app (`index.html`, `app.js`, worker, WASM).

## Run Locally

### 0) K-8 combo demo (AprilTag + body tracking)
```powershell
cd c:\src\apriltags
.\venv\Scripts\Activate.ps1
pip install mediapipe
python apriltag_human_combo_demo.py
```

This opens one camera view that overlays:
- AprilTag ID + distance + orientation
- Full human skeleton tracking
- A simple "hands up" interaction cue for classroom demos

### 1) Python scripts (local webcam + OpenCV)
```powershell
cd c:\src\apriltags
.\venv\Scripts\Activate.ps1
python apriltag_pose_logger.py
```

Press `q` to quit. Output logs are written to `apriltag_log.csv`.

### 2) Browser detector (no install for end users)
```powershell
cd c:\src\apriltags\web
npx serve .
```

Open the local URL in your browser and click **Start camera**.

## Deploy (Vercel)
- Set project **Root Directory** to `web`.
- Use framework preset **Other**.
- Keep build/install overrides empty.
- Ensure `web/vercel.json` is present (WASM header).

## Tech Stack
- Python, OpenCV, NumPy, pupil-apriltags
- JavaScript, Web Workers, WebAssembly, Comlink
- Vercel (static hosting)
