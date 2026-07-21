# SLAM / Perception Pipeline

Branch: **`erim/perception`**

Unified perception stack for IARC Mission 10: AprilTag mine detection, obstacle mapping, and onboard localization. Runs on the Pi (mono USB cam) or a stereo camera setup.

---

## Quick start

### Pi — mines + fused localization (default)

```bash
git checkout erim/perception
bash SLAM/scripts/run_pi.sh
```

Requires:
- `apriltags/camera_calib.npz` (run `bash SLAM/scripts/calibrate_pi.sh` first)
- ESP32 on WiFi with **updated firmware** (see Firmware section)
- `esp32_host` set in `SLAM/pipeline_config.pi.json`

### Stereo — mines + trees/obstacles

```bash
bash SLAM/scripts/run_stereo.sh
```

Uses `pipeline_config.stereo.json` (2560×720 side-by-side cam, obstacles enabled).

### Offline localization test (no hardware)

```bash
cd SLAM
python scripts/test_localization_offline.py
```

---

## What it does

```
Camera frame
    │
    ├─► AprilTag detector ──► mine world position ──► 2D mine map (human path)
    │
    ├─► Obstacle detector (stereo only) ──► tree/obstacle voxels ──► 3D flight map
    │
    └─► Visual odometry + ESP32 IMU ──► fused pose ──► COM_SET_ST_EST ──► ESP32
```

### Mine detection (AprilTags)

- Detects `tag36h11` AprilTags on mines
- Transforms tag pose to world coordinates using drone pose + camera extrinsics
- Fuses repeated sightings per tag ID (Kalman + confidence weighting)
- Writes to **2D permanent mine layer** on the shared map
- Output: `mine_detections.csv`, right panel of `occ_grid_proj.png`

### Obstacle detection (trees, stereo cam)

- YOLO + stereo depth (or depth-cluster fallback)
- Targets configurable classes (default `"tree"`) — needs custom YOLO weights for real tree labels
- Fuses obstacles by proximity in world space
- Writes to **3D permanent obstacle layer** (separate from mines — not used for human path)
- Multi-drone sharing via `shared_obstacle_map.json`
- **Requires stereo camera** — mono Pi cam cannot do depth obstacles

### Localization (new)

The ESP32 previously returned `(0,0,0)` for position because state estimation was never implemented. The Pi now runs onboard fusion and **pushes pose to the ESP32** via `COM_SET_ST_EST`.

**Sources fused on the Pi:**

| Source | What it provides |
|--------|------------------|
| Launch pose | Known origin at start line `(0, 0, height)` |
| ESP32 IMU (MPU6050) | Roll/pitch/yaw via complementary filter |
| Visual odometry | Lateral motion from camera optical flow |
| AprilTag loop closure | Re-seen tags nudge pose toward fused mine positions |

**Flow each frame:**
1. Pi requests IMU from ESP32 (`COM_REQUEST_SENSORS`)
2. Pi runs optical-flow VO on camera frame
3. `PoseFusion` updates position + attitude
4. Pi pushes fused state to ESP32 (`COM_SET_ST_EST`) every ~100ms
5. Mine/obstacle detections use fused pose for world coordinates

---

## Map architecture

One world coordinate system (meters), two layers in `SparseVoxelMap`:

| Layer | Storage | Used for |
|-------|---------|----------|
| **Mines** | 2D `permanent_cells` | Human path planning |
| **Obstacles** | 3D `permanent_obstacle_voxels` | Drone flight avoidance |

Field default: **94 m × 12 m**, cell size **0.2 m**.

Visualization: `occ_grid_proj.png`
- Left = 3D obstacles (drone nav)
- Right = 2D mines (human path)

---

## Configuration

### Config files

| File | Use case |
|------|----------|
| `pipeline_config.json` | Laptop dev, mono, obstacles off |
| `pipeline_config.pi.json` | Pi deployment, fused pose |
| `pipeline_config.stereo.json` | Stereo cam, mines + obstacles |

### Pose sources (`pose_source`)

| Value | Description |
|-------|-------------|
| `stub` | Fixed position for testing without drone |
| `esp32` | Read position from ESP32 (only works if something sets `stateEstimate`) |
| `fused` | **Recommended** — Pi IMU + VO fusion, pushes to ESP32 |

### Localization block (in JSON)

```json
"localization": {
  "launch_position_m": [0.0, 0.0, 1.5],
  "launch_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
  "vo_altitude_m": 1.5,
  "pose_push_interval_s": 0.1,
  "tag_correction_gain": 0.25
}
```

- `launch_position_m` — field origin at deployment (start line)
- `vo_altitude_m` — flight height for VO scale (set to actual hover height)
- `tag_correction_gain` — how aggressively re-seen AprilTags correct drift (0–1)

### Key mine settings

- `tag_size_m` — physical black square width in meters (measure with ruler)
- `calib_file` — path to `apriltags/camera_calib.npz`
- `drone_camera` — camera mount offset from drone body

---

## Camera calibration

The pipeline will not start without `apriltags/camera_calib.npz`.

```bash
# Find camera index
python SLAM/scripts/scan_cameras.py --width 1280 --height 720

# Calibrate (needs display + printed chessboard, 9×6 inner corners)
bash SLAM/scripts/calibrate_pi.sh
```

Calibrate at the **same resolution** you fly at. Re-calibrate if you change camera or resolution.

---

## Firmware requirements

Flash ESP32 from `ESP32/KAF_Drone/` on `erim/perception`. New/changed:

- **`COM_REQUEST_SENSORS` (0x66)** — Pi reads MPU6050 accel + gyro
- **`COM_SET_ST_EST` (0x60)** — Pi writes fused pose (already existed, now used)
- **IMU attitude backup** in `flight_task.cpp` — ESP32 runs its own complementary filter

Without updated firmware, fused localization cannot read IMU data.

---

## Outputs

| File | Contents |
|------|----------|
| `mine_detections.csv` | Per-detection log |
| `pipeline_stats.log` | FPS, mine count, pose, corrections |
| `obstacle_map.json` | Fused obstacle positions |
| `shared_obstacle_map.json` | Multi-drone obstacle sharing |
| `occ_grid_proj.png` | Map visualization |
| `occ_grid_2d.npy` / `occ_grid_3d.npy` | Raw grid arrays |

---

## CLI flags

```bash
python SLAM/mine_detection_pipeline.py \
  --config SLAM/pipeline_config.pi.json \
  --pose-source fused \
  --esp32-host 192.168.x.x \
  --visualize
```

| Flag | Effect |
|------|--------|
| `--pose-source fused` | Enable onboard localization |
| `--pose-source stub` | Fixed test pose |
| `--pose-source esp32` | Read pose from ESP32 only |
| `--stereo` | Enable stereo mode + obstacles |
| `--headless` | No display (auto on Pi without monitor) |
| `--visualize` | Show detection overlay |

---

## Bench validation (do this before competition)

1. Flash updated ESP32 firmware
2. Calibrate camera at flight resolution
3. Set `esp32_host` and `tag_size_m` in config
4. Run pipeline, move drone 2 m — check `pipeline_stats.log` for changing pose
5. Point at AprilTag at known distance — check world coords are reasonable
6. Measure error at 10 m, 30 m, 60 m — tune `vo_altitude_m` and `tag_correction_gain`

**Expected accuracy (rough):**
- Short sorties with tag re-visits: ~0.2–0.5 m on mine positions
- Full field traverse without revisits: 1–3 m drift possible
- Use conservative mine inflation on human path to compensate

---

## Repo layout

```
SLAM/
├── mine_detection_pipeline.py   # Main entry (PerceptionPipeline)
├── sparse_voxel_map.py          # Shared map (mines + obstacles)
├── apriltag/                    # AprilTag detection + mine fusion
├── obstacle/                    # Stereo YOLO/depth obstacle detection
├── localization/                # IMU + VO fusion + ESP32 comms
│   ├── esp32_comms.py           # TCP protocol (SET_ST_EST, sensors)
│   ├── imu.py                   # Attitude filter
│   ├── visual_odometry.py       # Optical flow VO
│   ├── fusion.py                # Pose fusion + loop closure
│   └── fused_pose_provider.py   # PoseProvider for pipeline
├── scripts/
│   ├── run_pi.sh
│   ├── run_stereo.sh
│   ├── calibrate_pi.sh
│   ├── scan_cameras.py
│   └── test_localization_offline.py
└── pipeline_config*.json
```

---

## What still needs work

- [ ] Field-test fused localization on real drone + measure drift
- [ ] Custom YOLO weights for tree class (default `yolov8n` has no "tree")
- [ ] Human path planner reading 2D mine map
- [ ] Flutter ground station mine map display
- [ ] UWB inter-drone ranging for map sharing (firmware has ranging, no fusion yet)

---

## IARC localization notes

- Arena provides **no localization infrastructure**
- Ground anchors at field edge give bad geometry for 91 m downrange — not recommended
- Drone-carried UWB for **inter-drone ranging** is viable (onboard, no denied access)
- Pi fusion from known launch pose + IMU + camera is the primary localization path
- Corner GPS at arrival can define field frame; onboard fusion handles position during flight
