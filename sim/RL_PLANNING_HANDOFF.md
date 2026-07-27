# WAAR IARC M10 — Codebase & Mission Sim Handoff (for RL Planning)

This document is a self-contained briefing for planning reinforcement learning (or other optimization) on top of the **`sim/`** package. It also explains how that sim relates to the rest of the repository (perception, firmware, ground station, and the older grid-based simulator under `src/`).

**Audience:** An engineer or LLM designing RL without having the repo open.

**Branch context:** Mission sim work lives on **`erim/sim`** (may be unmerged). Perception/localization on **`erim/perception`**. Main README is minimal (`# drone_code`).

---

## 1. Competition problem (what we are optimizing toward)

**IARC Mission 10 (simplified):**

- Arena **91.44 m × 24.38 m** (300 ft × 80 ft; configurable in sim).
- **Mines** are marked with **AprilTags** (`tag36h11`). Drones must **discover** mine positions and build a map.
- A **human** must later walk across the field; the team must plan a **safe path** for the human that avoids known mines (with clearance / inflation).
- **Four drones** operate in the same world frame.
- **Trees/obstacles** matter for **drone flight** (3D layer in SLAM), not for the 2D human path map in the current sim.
- **Localization:** Pi fuses IMU + visual odometry + AprilTag loop closure and pushes pose to ESP32 (`COM_SET_ST_EST`). Ground UWB anchors are not used for absolute field position.

**Two planning horizons:**

| Horizon | Question | Current code |
|--------|----------|--------------|
| **Exploration** | Where should drones fly to discover mines quickly? | `sim/exploration.py` + placeholder patrol |
| **Human path** | Given discovered mines, is there a corridor start→goal? | `sim/pathfinding.py` + `HumanPathField` |
| **Joint / RL** | Policies that trade off coverage, time, altitude, multi-drone coordination, and eventual human-path quality | **Not implemented** — target of next work |

---

## 2. Repository map (high level)

```
waar-iarc-m10/
├── sim/                    # Mission sim v1/v2 — human path + exploration (THIS DOC FOCUS)
├── SLAM/                   # Real perception: AprilTag mines, obstacles, fused pose, Pi scripts
├── apriltags/              # Tag detector assets, WASM demo, calibration helpers
├── ESP32/KAF_Drone/        # On-drone firmware: motors, COM protocol, partial flight modes
├── Ground Station/esp32_split2/  # Flutter app: TCP teleop, motor mixer, kill
├── src/                    # Separate grid-world sim (Max): corridor certification, multi-agent
├── Hardware/               # KiCad schematics
└── README.md               # Stub
```

**Important:** `sim/` and `src/` are **not wired together**. RL can either extend `sim/` (continuous meters, 4 drones, human A*) or port ideas from `src/` (frontier scoring, corridor certification). For IARC-shaped metrics, **`sim/` is the right host**.

---

## 3. What flies autonomously today (hardware truth)

This bounds what “realistic” sim actions should look like.

| Layer | Status |
|-------|--------|
| **Flutter GS** | Sends **motor mix** at 250 ms (`roll/pitch/yaw/throttle` → 4 floats, `COM_SET_MOTOR_CMD`) |
| **ESP32 `MOTOR_SETPOINT_MODE`** | Motors follow commanded mix |
| **ESP32 `POS_SETPOINT_MODE` / `TRAJECTORY_MODE`** | **Incomplete** (`flight_task.cpp` has `//wip` on attitude→motor) |
| **Pi / SLAM** | **Pose in only** (`COM_SET_ST_EST`), no trajectory commands to drone |
| **Mission executive** | **Missing** — no waypoint follower on Pi or ESP32 |

So for RL near-term:

- **Action space A:** 4D sticks `(throttle, pitch, roll, yaw)` each in `[-1,1]` or `[0,1]` for throttle — passed through `sim/drone_flight.py` mixer (matches GS).
- **Action space B (future):** `COM_SET_POS_CMD` once firmware PID is finished — sim already has stubs; not used in exploration yet.

---

## 4. Mission sim (`sim/`) — purpose and modes

Entry point: `python -m sim` → `sim/cli.py`.

### 4.1 Mode A — Instant human path (no time, no drones)

All mines known upfront → build grid → **A\*** from **start line** (near `x = edge_margin`) to **goal line** (near `x = field_x - edge_margin`) → PNG/JSON.

### 4.2 Mode B — Exploration over time (default for RL prep)

- **Truth:** full list of `Mine` objects (random, CSV, or JSON).
- **Belief:** `discovered` dict grows as drones “see” mines.
- **Human path:** recomputed with A* only on **discovered** mines (realistic: human path unknown until mapped).
- **Ticks:** discrete steps; each tick every drone moves once, then discovery + optional replan.

CLI flags (representative):

| Flag | Meaning |
|------|---------|
| `--explore` | Enable time-based exploration |
| `--drones N` | Default **4** |
| `--ticks` | Max steps |
| `--sensor-range` | Ground range at **ref altitude** (see perception) |
| `--default-altitude`, `--min-altitude`, `--max-altitude` | Flight envelope (m AGL) |
| `--animate` / `--no-animate` | Matplotlib live vs headless |
| `--legacy-patrol` | Skip flight model; old grid teleport |

---

## 5. Sim module — file-by-file

### `sim/config.py` — `MissionSimConfig`

| Field | Default | Role |
|-------|---------|------|
| `field_x_m`, `field_y_m` | 91.44, 24.38 | Long (downrange) × width (m); start/goal on width edges |
| `resolution_m` | 0.2 | Grid cell size for human map |
| `clearance_m` | 0.3 | Mine inflation radius |
| `edge_margin_m` | 0.5 | Start/goal lines inset |
| `ground_z_m` | 0 | Ground plane |
| `default_altitude_m` | 1.5 | Cruise / spawn z |
| `min_altitude_m`, `max_altitude_m` | 0.4, 3.0 | z clamps |

Derived: `rows`, `cols` for occupancy grid.

### `sim/types.py`

Cell values: `FREE`, `HAZARD`, `INFLATED`.

### `sim/mines.py` — `Mine`

```python
@dataclass(frozen=True)
class Mine:
    tag_id: int
    world_x: float
    world_y: float   # ground plane only; z=0 implicit
    confidence: float
```

Loaders: CSV (`SLAM/mine_detections.csv` schema), JSON, `generate_random_mines(seed)`.

### `sim/field.py` — `HumanPathField`

- **2D only** — human path planning ignores drone altitude and trees.
- `rebuild_from_mines()` clears grid, places `HAZARD` at mine cell, `INFLATED` in clearance disk.
- `start_cells()` / `goal_cells()` — vertical segments at left/right margins.

### `sim/pathfinding.py`

- **A\*** 8-connected on passable cells; optimize any start cell on start line → any goal cell on goal line.
- Heuristic: column distance to goal line.
- `path_length_m()` — physical length using cell centers.

### `sim/drone_flight.py` — physics + control (hardware-aligned)

**Flight mode constants** mirror `ESP32/KAF_Drone/src/communication.h`:

- `NULL_MODE`, `MOTOR_SETPOINT_MODE`, `POS_SETPOINT_MODE`, `TRAJECTORY_MODE`

**`mix_motors_from_sticks`:** Copy of Flutter `DroneController` quad-X mixer with `_curve_throttle` (sqrt) and `_curve_signed` (signed sqrt).

**`DroneFlightModel` state:**

| Variable | Meaning |
|----------|---------|
| `x, y, z` | World position (m); z is altitude AGL |
| `yaw, pitch, roll` | Heading (rad) and body tilt from sticks |
| `vx, vy, vz` | Velocities |
| `hover_throttle` | 0.5 (GS default) |
| `control_dt_s` | **0.25** s (250 ms GS loop) |

**Dynamics (motor mode):**

- Pitch/roll sticks → horizontal accel in body frame → rotate to world by `yaw`.
- Yaw stick → yaw rate.
- Throttle (shaped) vs shaped hover thrust → vertical accel → `vz`, integrate `z`, clamp `[min_z, max_z]`.
- Linear drag on velocities.
- World `x,y` clamped to field margins; velocity zeroed at walls.

**Altitude behavior today:** Patrol (`SerpentinePatrol`) sets `target_z_m = default_altitude` and adjusts throttle with P on `z_err`. At cruise, drones **hold ~1.5 m** — you will **not** see altitude change unless the policy commands sustained throttle bias or you add tasks that require climbing (RL can use z as action dimension).

**`SerpentinePatrol`:** Placeholder autonomy — waypoint serpentine in x/y, outputs sticks. **This is what RL should replace** for exploration (one policy per drone or centralized).

### `sim/perception_geometry.py` — `DroneSensorModel`

Mines detected if ground point inside **altitude-dependent footprint**:

- Parameters: `ref_altitude_m`, `ref_ground_range_m`, `camera_hfov_deg`, `camera_vfov_deg`, `camera_pitch_deg` (default -45°).
- `ground_footprint_radius_m(z)` — grows with altitude (FOV hook for RL observations).
- `can_detect_ground_point(...)` — horizontal distance + slant check.

**RL note:** Observation can include footprint radius, or a binary mask of “visible ground cells” at current pose.

### `sim/exploration.py` — `ExplorationSim`

**State:**

- `truth_mines`, `discovered: dict[tag_id, Mine]`
- `field: HumanPathField` — belief map for human path
- `path: list[(row,col)] | None` — current human path if discoverable set allows A*
- `drones: list[DroneState]`
- `sensor: DroneSensorModel`
- `_visited: set[(row,col)]` — coverage metric (2D grid cells overflown)
- `_last_replan_discovered` — replan human path only when `|discovered|` increases (perf)

**`DroneState`:**

- `flight: DroneFlightModel`
- `patrol: SerpentinePatrol`
- `index: int`
- `trail: list[(x,y,z)]`

**`step()` order per tick:**

1. For each drone: `_move_drone` (patrol → sticks → `controls_step`)
2. `_discover_near` (sensor model or legacy disk)
3. Mark visited cell from `(x,y)`
4. `_replan` human A* if new mines

**`ExplorationMetrics`:**

- `ticks`, `mines_discovered / mines_total`, `path_found`, `path_length_m`, `coverage_ratio`, `drone_altitudes_m` tuple
- Printed in `summary()` including `z_m=...`

**CSV replay:** `discover_from_csv_row` — no drone motion; mines appear in log order (`sim/replay.py`).

### `sim/visualize.py`

- **Top panel:** 2D occupancy + human path + drones + heading lines + **dashed footprint circles**
- **Bottom panel:** altitude vs downrange (x–z trails)
- `run_explore_animation` for live; `save_human_path_plot` for PNG

### `sim/cli.py`, `sim/__main__.py`

Argument parsing and wiring to `MissionSimConfig` + `ExplorationSim`.

### `sim/replay.py`

`mines_by_timestamp(csv)` → sorted unique tag first-seen events.

### `sim/README.md`

User-facing commands (may lag code slightly; trust this handoff for RL).

---

## 6. Tick semantics (for RL `env.step`)

Treat one call to `ExplorationSim.step()` as one **environment step**:

- **Δt:** `DroneFlightModel.control_dt_s` = **0.25 s** per drone control update (all drones step once per env step).
- **Simultaneous:** All drones move in parallel within the same tick (no intra-tick ordering — may matter for multi-agent RL; could add sequential phase later).

**Suggested Gym-style wrapper:**

```text
reset(seed) → load mines, reset drones at start line lanes, clear discovered
step(action) → action shape (num_drones, 4) or dict per drone → set_sticks → step()
obs → belief map tensor, drone poses, discovered count, path_exists, ...
reward → user-defined (see §9)
done → all mines found OR max ticks OR human path certified
```

---

## 7. SLAM / perception (real world ↔ sim)

Path: `SLAM/mine_detection_pipeline.py` (`PerceptionPipeline`).

- **Mines:** AprilTags → world XY → `MineRegistry` → 2D map.
- **Obstacles:** `SLAM/obstacle/` stereo + YOLO (separate 3D layer).
- **Pose:** `pose_source`: `stub` | `esp32` | **`fused`** (Pi IMU + VO + tag loop closure).
- **Configs:** `pipeline_config.pi.json`, `pipeline_config.stereo.json`.
- **Outputs:** `mine_detections.csv`, `occ_grid_proj.png`, shared JSON maps.

**Sim ↔ SLAM alignment:**

| Sim | SLAM |
|-----|------|
| `Mine.world_x/y` | CSV `world_x`, `world_y` |
| `HumanPathField` 2D grid | `SparseVoxelMap` mine layer |
| `DroneSensorModel` | Simplified stand-in for camera FOV + range |
| `DroneFlightModel` pose | Fused pose / `stateEstimate` |

RL trained in sim can later use **CSV replay mode** to validate discovery ordering against real logs (`--replay-csv`).

---

## 8. ESP32 + Ground Station (control interface)

**Protocol:** `ESP32/KAF_Drone/src/communication.h`, Dart `Ground Station/.../drone_protocol.dart`.

Relevant commands:

- `COM_SET_MOTOR_CMD` — 4 floats 0–1
- `COM_SET_CTRL_MODE` — `MOTOR_SETPOINT_MODE` (0x02), etc.
- `COM_SET_POS_CMD` — position setpoint (firmware incomplete)
- `COM_SET_ST_EST` — full state estimate from Pi
- `COM_KILL`

**Teleop:** `drone_controller.dart` — arm/disarm, 250 ms motor packets.

Sim intentionally mirrors **motor path**, not unfinished position loops.

---

## 9. Legacy grid simulator (`src/`) — optional RL ideas

Hex/block grid, **not** meter-accurate IARC field.

- `src/application/simulator.py` — observe → corridor → certify → frontier
- `src/multi_agent/coordinator.py` — multiple drones, collision avoidance on **blocks**
- `src/use_cases/*` — corridor-aware frontier scoring
- **Certification:** corridor clearance + coverage thresholds

Useful concepts to **port** into `sim/` RL reward:

- Bonus for maintaining a **valid human corridor** with clearance
- Frontier / unknown coverage (sim already has `_visited` coverage ratio)
- Multi-agent collision penalty (not in `sim/` yet — drones can overlap)

---

## 10. RL problem framing (suggested, not implemented)

### 10.1 Objectives (multi-criteria)

1. **Discover all mines** (or maximize count by time T).
2. **Enable human path** — `path_found` and minimize `path_length_m` subject to clearance.
3. **Time** — minimize ticks.
4. **Coverage** — maximize `coverage_ratio` or weighted area of footprint union.
5. **Altitude / FOV tradeoff** — higher z → larger `ground_footprint_radius_m` but worse resolution (can add penalty or explicit tag detection probability later).
6. **Multi-drone** — de-duplicate discoveries; avoid redundant overlap; optional inter-drone distance penalty (UWB ranging exists in firmware but not in sim).

### 10.2 Observation space (candidates)

**Global (centralized policy):**

- Downsampled belief grid: unknown / free / hazard (only discovered mines).
- Drone states: `(x, y, z, yaw, vx, vy, vz)` × 4.
- Scalar: `mines_discovered`, `path_found`, `path_length_m`.

**Decentralized (per drone):**

- Ego pose + partial map crop around drone.
- Footprint radius at current z.
- Nearest undiscovered mine direction (cheating if used at deploy — use for curriculum only).

**Privileged (training only):**

- Full truth mine locations for reward shaping.

### 10.3 Action space (candidates)

| Space | Dim | Notes |
|-------|-----|------|
| Sticks | 4 × N drones | Matches hardware; use `DroneFlightModel.set_sticks` |
| Motor | 4 × N | `set_motor_cmd` bypass shaping |
| Waypoint delta | 3 × N | Would need position controller in sim (implement POS mode properly) |
| Discrete macro | K actions | e.g. “move to next lattice point”, “climb 0.2 m” — easier but less hardware-faithful |

### 10.4 Reward sketch

```text
r_t = α * (new_mines_discovered)
    + β * (path_found_bonus if path just became feasible)
    - γ * Δt
    + δ * (coverage_gain)
    - ε * separation_reward_penalty(...)   # sim/separation.py; R_soft shaping + R_hard spike
```

Use `ExplorationSim.compute_drone_separation()` or `metrics.separation_*` after each step.
`separation_reward_penalty(snap, r_soft_m=config.min_separation_soft_m, r_hard_m=config.min_separation_hard_m)`.

Terminal: all mines found + path exists; or large bonus for “mission ready”.

### 10.5 Integration points in code

| Hook | Location |
|------|----------|
| Replace patrol | `ExplorationSim._move_drone` — call policy instead of `SerpentinePatrol.sticks_for_pose` |
| Custom discovery | `ExplorationSim._discover_near` — add noise, false positives, tag confidence |
| Env API | New `sim/rl_env.py` wrapping `ExplorationSim` (recommended) |
| Vectorized reset | `generate_random_mines` + seed in `reset()` |
| Logging | Extend `ExplorationMetrics` or sidecar for episode returns |

### 10.6 What is **not** in sim yet (RL design should account for)

- Drone–drone collision / UWB ranges
- Obstacle map / trees (3D) — only in SLAM, not `HumanPathField`
- Wind, battery, motor latency beyond fixed 250 ms
- False AprilTag detections / pose noise
- Communication limits between drones
- **Position-mode autopilot** on real hardware

---

## 11. Altitude and FOV (user-visible behavior)

- **Capability:** `z`, `vz`, throttle-based climb, clamps, sensor footprint ∝ f(z).
- **Default patrol:** holds `target_z_m = default_altitude_m` → **flat altitude vs x** in bottom panel (expected).
- **RL should explicitly command throttle** (or target z) to exploit altitude–FOV tradeoff.

Formula reference:

```text
radius ≈ max( f(altitude, hfov, vfov, pitch), ref_ground_range * (z / ref_altitude) * 0.5 )
```

See `DroneSensorModel.ground_footprint_radius_m` in `sim/perception_geometry.py`.

---

## 12. Key commands

```bash
# Exploration, 4 drones, live UI
python -m sim --explore --random-mines 20 --animate --show-truth

# Headless episode
python -m sim --explore --random-mines 20 --ticks 400 --no-animate

# Single drone ablation
python -m sim --explore --drones 1 --random-mines 15 --no-animate

# Real log ordering (no flight)
python -m sim --explore --replay-csv SLAM/mine_detections.csv --no-animate

# Instant human path (upper bound on path length)
python -m sim --random-mines 20 --seed 42
```

---

## 13. Dependencies

- Python 3.x, **numpy**, **matplotlib** (Agg for CI: `MPLBACKEND=Agg`).
- No PyTorch/Gym installed by default — RL stack is greenfield.

---

## 14. Suggested RL implementation plan (phased)

1. **`sim/rl_env.py`** — Gymnasium `Env` with `reset/step/render`, wraps `ExplorationSim`, actions → sticks.
2. **Reward only** — discover mines + time penalty; verify learning beats `SerpentinePatrol`.
3. **Add human-path shaping** — reward when `path_found` flips true; penalize long `path_length_m`.
4. **Altitude in action/obs** — curriculum: allow throttle bias; reward footprint coverage efficiency.
5. **Multi-agent** — parameter sharing or CTDE; add collision penalty in sim.
6. **Sim2real gap** — pose noise, CSV replay evaluation, later hook to `PerceptionPipeline` synthetic camera.

---

## 15. File index (sim only)

| File | Role |
|------|------|
| `config.py` | Field + altitude config |
| `types.py` | Grid cell enums |
| `mines.py` | Mine dataclass + loaders |
| `field.py` | Human 2D occupancy |
| `pathfinding.py` | Human A* |
| `drone_flight.py` | Motor mixer + 3D kinematics |
| `perception_geometry.py` | Altitude-dependent detection |
| `exploration.py` | Main simulation loop |
| `separation.py` | Inter-drone distance + RL penalty helper |
| `visualize.py` | 2D + altitude UI |
| `cli.py` | CLI |
| `__main__.py` | `python -m sim` |

---

*Generated for RL planning handoff. Update this doc when adding `rl_env.py` or changing tick/action contracts.*
