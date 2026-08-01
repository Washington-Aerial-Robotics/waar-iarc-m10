# WAAR Autonomy — Single-Drone Corridor Explorer

A simulation of a single drone exploring an unknown hazardous environment and certifying a safe passage corridor from start to goal. The codebase follows **Clean Architecture** so every algorithmic layer is independently testable and replaceable.

---

## Setup

```bash
conda env create -f environment.yml
conda activate waar-autonomy
```

---

## Run

```bash
python sim.py                  # animated (default seed 42)
python sim.py --seed 7         # different map
python sim.py --no-anim        # headless, prints progress to console
python sim.py --hazards 40     # denser hazard field
python sim.py --delay 0.02     # slow down animation frame rate
```

---

## What the Simulation Does

The drone starts in the bottom-left corner of an unknown grid and must find and certify a **safe corridor** to the top-right goal.

### Three-Map Architecture

| Map | Grid scale | Contents | Accessible to |
|-----|-----------|----------|---------------|
| **Map 1** — Ground truth | Fine (`80×60`) | True hazard positions | Infrastructure only — hidden from planner |
| **Map 2** — Visited blocks | Block (`20×15`) | Blocks the drone has entered | Domain `WorldModel` |
| **Map 3** — Detected map | Fine (`80×60`) | `UNKNOWN / FREE / HAZARD / INFLATED` | All use cases |

The fine grid is `K=4` cells per block side, so a `20×15` block grid becomes an `80×60` fine grid.

### Cell States (Map 3)

| State | Colour | Meaning |
|-------|--------|---------|
| `UNKNOWN` | Dark | Not yet visited |
| `FREE` | Light grey | Visited, no hazard |
| `HAZARD` | Red | Confirmed hazard cell |
| `INFLATED` | Orange | Safety buffer around a hazard (radius 3.5 fine cells) |

### Per-Tick Loop

```
UC1  observe_block    → reveal K×K fine cells from Map 1 into Map 3
                        recompute inflation buffer if a hazard was found
UC2  compute_corridor → A* from start to goal on Map 3
                        UNKNOWN cells cost 4× (optimistic planning)
UC3  certify?         → corridor is certified if:
                          clearance ≥ 2.0 fine cells from nearest hazard
                          coverage  = 100% FREE cells (no UNKNOWN on path)
UC4  score_frontiers  → rank unvisited border blocks by:
                          −(travel_cost + goal_cost) + 4.0 × cert_gain
                        cert_gain = UNKNOWN cells on current corridor path
     move drone       → teleport to highest-scoring frontier block
```

---

## Project Structure

```
sim.py                              ← CLI entry point (thin wrapper)

src/
  domain/
    types.py                        ← K, cell states, block↔fine helpers
    world_model.py                  ← WorldModel: map2 (visited) + map3 (detected)
    drone_state.py                  ← DroneState: block + fine-cell position

  ports/
    ground_truth_port.py            ← Protocol: is_hazard(fx, fy)

  adapters/
    sim_ground_truth_adapter.py     ← Wraps GroundTruthMap for the port

  use_cases/
    types.py                        ← CertificationResult DTO
    update_world_model.py           ← UC1: observe block + recompute inflation
    compute_best_corridor.py        ← UC2: A* on fine grid
    evaluate_and_certify_corridor.py← UC3: clearance + coverage check
    score_frontiers_corridor_aware.py← UC4: reverse-Dijkstra + frontier scoring

  application/
    simulator.py                    ← Simulator: wires UC1→UC4 per tick

  infrastructure/
    env/ground_truth_map.py         ← GroundTruthMap (map1, hidden)
    env/map_factory.py              ← Seeded random hazard placement
    visualization/renderer.py       ← build_truth_image / build_detected_image

  experiments/
    config.py                       ← Config dataclass (all tunable parameters)
    run_sim.py                      ← run_headless, run_animated, main()

tests/                              ← Mirrors src/ layout, 43 tests
```

### Layer Import Rules

```
domain/       →  no imports from any other src/ layer
ports/        →  domain/ only
use_cases/    →  domain/, ports/ only
adapters/     →  domain/, ports/, infrastructure/
application/  →  all layers
experiments/  →  all layers (entry points)
```

---

## Configuration

All parameters live in [src/experiments/config.py](src/experiments/config.py):

| Parameter | Default | Effect |
|-----------|---------|--------|
| `block_cols` / `block_rows` | 20 / 15 | Grid dimensions in blocks |
| `n_hazards` | 25 | Number of hazard cells placed at random |
| `inflation_radius` | 3.5 | Safety buffer radius (fine cells) |
| `unknown_cost` | 4.0 | A* penalty per UNKNOWN fine cell |
| `min_clearance_cells` | 2.0 | Certification: min distance from any hazard |
| `min_coverage_ratio` | 1.0 | Certification: fraction of path cells that must be FREE |
| `w_cert` | 4.0 | Scoring weight for corridor-overlap uncertainty |
| `max_ticks` | 600 | Simulation budget |

---

## Tests

```bash
pytest              # 43 tests across all layers
pytest tests/domain/
pytest tests/use_cases/
pytest tests/application/   # includes full integration test (seed 42, tick 263)
```

---

## Example Output (headless)

```
Seed=42  blocks=20x15  fine=80x60  K=4  hazards=25
  tick=  50  corridor=yes  clearance=3.61  coverage=94%
  tick= 100  corridor=yes  clearance=3.61  coverage=94%
  tick= 150  corridor=yes  clearance=3.61  coverage=94%
  tick= 200  corridor=yes  clearance=3.61  coverage=95%
  tick= 250  corridor=yes  clearance=3.61  coverage=90%
  tick= 263  corridor=yes  clearance=3.61  coverage=100%
elapsed=4.64s

✅  CERTIFIED at tick 263  clearance=3.61 fine-cells  path_len=93 fine-cells
```
