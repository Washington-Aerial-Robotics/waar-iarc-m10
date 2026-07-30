# Mission sim

IARC arena (default **91.44 m × 24.38 m**, 300 ft × 80 ft), 2D mine map with **1 ft (0.3048 m)** inflation, **max-width** human path from start edge to far edge over **discovered mines only**.

## v1 — instant plan

```bash
python -m sim --random-mines 20 --seed 42
python -m sim --mines-csv SLAM/mine_detections.csv --min-confidence 0.2
python -m sim --random-mines 15 --export-path sim/human_path.json
```

## v2 — exploration + UI

Drones patrol and **discover** mines; path replans as the map grows. **3D state** (`x`,`y`,`z`) with motor-mixer flight; viewer shows top-down map (dashed circle = ground footprint) + altitude vs downrange. See `sim/perception_geometry.py` for altitude-dependent detection (FOV hooks for RL).

```bash
# Headless exploration (saves PNG at end; default run length = 7 min survey)
python -m sim --explore --random-mines 20 --no-animate

# Live viewer — grey undiscovered mines, HUD timer, ~14× playback (7 min ≈ 30 s wall)
python -m sim --explore --random-mines 20 --animate

# Realtime playback (physics still 0.25 s/tick; one render per tick)
python -m sim --explore --random-mines 20 --animate --time-scale 1

# Replay perception log in time order (no drone motion)
python -m sim --explore --replay-csv SLAM/mine_detections.csv --animate
```

| Flag | Description |
|------|-------------|
| `--explore` | Time-based discovery instead of all mines at once |
| `--animate` | Live window (close when done; saves `--output`) |
| `--time-scale N` | Playback only: pause `0.25/N` s per tick (default **14**). Smooth — every tick is drawn. Does **not** change drone speed |
| `--show-truth` / `--hide-truth` | Light-grey X for undiscovered mines (default **on**) |
| `--search-speed` | Lane cruise speed m/s (default **2.0**, tunable) |
| `--ticks` | Max physics ticks (default **1680** = 7:00) |
| `--drones` | Patrol drones (default **4**; use `--drones 1` for single-drone tests) |
| `--sensor-range` | Ground detection radius at ref altitude (scales with height) |
| `--default-altitude` | Cruise altitude in meters AGL (default 1.5) |
| `--min-separation-soft` / `--min-separation-hard` | RL shaping (4 m) vs safety metric (1.5 m) |
| `--replay-csv` | Add mines as they first appear in `mine_detections.csv` |
| `--serpentine-patrol` / `--legacy-patrol` | Fallback coverage controllers |

Metrics printed at end: `time=mm:ss / 07:00`, `mines found`, `path yes/no`, `path length`, `grid coverage`.

## Outputs

- `sim/human_path.png` — mine map + path (+ drones / truth in explore mode)
- Optional `sim/human_path.json` waypoints

## Later (phase 3)

- Synthetic camera + perception pipeline hook
- Flutter ground station map (separate from this package)
