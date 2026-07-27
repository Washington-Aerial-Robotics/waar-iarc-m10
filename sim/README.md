# Mission sim

IARC arena (default **91.44 m × 24.38 m**, 300 ft × 80 ft), 2D mine map with inflation, **A\*** human path from start line to goal line.

## v1 — instant plan

```bash
python -m sim --random-mines 20 --seed 42
python -m sim --mines-csv SLAM/mine_detections.csv --min-confidence 0.2
python -m sim --random-mines 15 --export-path sim/human_path.json
```

## v2 — exploration + UI

Drones patrol and **discover** mines; path replans as the map grows. **3D state** (`x`,`y`,`z`) with motor-mixer flight; viewer shows top-down map (dashed circle = ground footprint) + altitude vs downrange. See `sim/perception_geometry.py` for altitude-dependent detection (FOV hooks for RL).

```bash
# Headless exploration (saves PNG at end)
python -m sim --explore --random-mines 20 --ticks 500 --show-truth --no-animate

# Live matplotlib viewer
python -m sim --explore --random-mines 20 --animate --show-truth --delay 0.02

# Replay perception log in time order (no drone motion)
python -m sim --explore --replay-csv SLAM/mine_detections.csv --animate
```

| Flag | Description |
|------|-------------|
| `--explore` | Time-based discovery instead of all mines at once |
| `--animate` | Live window (close when done; saves `--output`) |
| `--show-truth` | Gray X for mines not yet discovered |
| `--drones` | Patrol drones (default **4**; use `--drones 1` for single-drone tests) |
| `--sensor-range` | Ground detection radius at ref altitude (scales with height) |
| `--default-altitude` | Cruise altitude in meters AGL (default 1.5) |
| `--min-separation-soft` / `--min-separation-hard` | RL shaping (4 m) vs safety metric (1.5 m) |
| `--replay-csv` | Add mines as they first appear in `mine_detections.csv` |

Metrics printed at end: `mines found`, `path yes/no`, `path length`, `grid coverage`.

## Outputs

- `sim/human_path.png` — mine map + path (+ drones / truth in explore mode)
- Optional `sim/human_path.json` waypoints

## Later (phase 3)

- Synthetic camera + perception pipeline hook
- Flutter ground station map (separate from this package)
