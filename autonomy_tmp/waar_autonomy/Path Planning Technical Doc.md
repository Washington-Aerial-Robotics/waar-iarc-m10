# WAAR Autonomy — Technical Handover Document

**Author:** *Kevin*  
**Scope:** Single-drone corridor exploration simulation (`sim.py`) and supporting architecture  

---

## 0. Purpose

This document records the design decisions, implementation details, and open work items for the single-drone autonomy module. It is intended to provide sufficient context for an incoming engineer to maintain and extend the codebase without loss of intent.

Multi-agent coordination (UC5, UC6) and mission-level planning are out of scope for this document and are covered separately by other team members.

---

## 1. Problem Statement

The drone operates in an unknown environment containing landmine-like hazards. The objective is to find and certify a safe traversal corridor from a start region to a goal region as quickly as possible, subject to the following constraints:

- The corridor must maintain a minimum clearance from all detected hazards.
- Every cell on the certified path must be confirmed free (no remaining uncertainty).
- The total mission budget is 600 ticks in simulation.

The fundamental planning tension is between **broad exploration** (reducing global uncertainty) and **focused certification** (resolving uncertainty specifically along the most promising corridor candidate). The scoring strategy described in §3.4 is the core contribution addressing this tension.

---

## 2. Implementation Scope

This work covers the single-drone planning loop only.

| Responsibility | Status |
|----------------|--------|
| World model maintenance (two-scale maps) | Implemented |
| Corridor search via A\* on fine cell grid | Implemented |
| Corridor certification (clearance + coverage) | Implemented |
| Corridor-aware frontier scoring (Reverse Dijkstra + `cert_gain`) | Implemented |
| Hazard inflation on fine cell grid | Implemented |
| Multi-agent assignment (UC5, UC6) | Out of scope — other team member |
| SLAM / pose estimation | Out of scope — external system |
| Hazard detection model | Out of scope — external system |
| Flight controller | Out of scope — external system |

---

## 3. Design Decisions

### 3.1 Two-Scale World Representation

The world model maintains two map resolutions concurrently.

| Map | Scale | Grid | Purpose | Accessible to |
|-----|-------|------|---------|---------------|
| Map 1 — Ground truth | Fine | 80 × 60 | True hazard positions | `infrastructure/` only — hidden from planner |
| Map 2 — Visited blocks | Coarse | 20 × 15 blocks | Frontier detection, visited tracking | Domain `WorldModel` |
| Map 3 — Detected map | Fine | 80 × 60 (K = 4 cells/block side) | Hazard state, inflation zones, A\*, clearance metrics | All use cases |

**Rationale.** Running A\* at block resolution produces clearance measurements too coarse for safety guarantees — a hazard can fall between block centres and go undetected by the path planner. Running frontier scoring at fine-cell resolution is computationally wasteful for assignment decisions. The two-scale design confines precise safety reasoning to the fine grid while keeping assignment decisions at block scale (O(1) lookup per frontier).

The bridge between scales is a single Reverse Dijkstra pass from the goal over Map 3, executed once per tick. Every frontier block's `goal_cost` is retrieved by looking up the cost at its block-centre fine cell — no per-block A\* is required.

### 3.2 A\* Cost Model with Optimistic UNKNOWN Penalty

A\* operates on Map 3 with the following cell costs:

| Cell State | Cost |
|------------|------|
| `FREE` | 1.0 |
| `UNKNOWN` | 4.0 |
| `HAZARD` | ∞ |
| `INFLATED` | ∞ |

**Rationale.** Assigning `UNKNOWN` a cost of ∞ prohibits path planning through unexplored space, forcing the drone to achieve full coverage before committing to any corridor — an overly conservative strategy incompatible with the time budget. Assigning `UNKNOWN` a cost equal to `FREE` (1.0) produces corridor candidates that route indiscriminately through unmapped regions. The value 4.0 implements optimistic planning: the planner commits to a corridor hypothesis through unknown space while maintaining a preference for confirmed-free paths. This parameter (`unknown_cost` in `Config`) is a primary tuning target.

### 3.3 Hazard Inflation into UNKNOWN Cells

On hazard detection, a circular inflation zone of radius 3.5 fine cells is immediately written as `INFLATED` (cost ∞) on Map 3. Inflation propagates into `UNKNOWN` neighbours, not only confirmed cells.

**Rationale.** Restricting inflation to confirmed cells would allow A\* to route through a buffer zone that may contain additional hazard extent due to positional uncertainty in the detection. Propagating into `UNKNOWN` neighbours ensures the path planner reroutes conservatively before those cells have been physically visited, preventing the drone from committing to a corridor that a subsequent observation would invalidate.

### 3.4 Corridor-Aware Frontier Scoring

Frontier blocks are scored by:

```
score(block) = −(travel_cost + goal_cost) + w_cert × cert_gain
```

| Term | Definition |
|------|-----------|
| `travel_cost` | Euclidean distance from the drone's current fine-cell position to the block centre. |
| `goal_cost` | Reverse Dijkstra cost from the block centre to the goal, precomputed on Map 3. |
| `cert_gain` | Count of `UNKNOWN` fine cells within this block that lie on the current A\* corridor path. |

**Rationale.** Pure goal-biased exploration moves the drone efficiently toward the goal but provides no incentive to resolve uncertainty along the current corridor candidate. A strategy that maximises `cert_gain` exclusively converges on the present corridor at the expense of discovering better alternatives. The combined formula is self-regulating: when `cert_gain` along the current corridor approaches zero (coverage near 100%), `travel_cost + goal_cost` dominates and the drone advances toward new frontiers. When A\* identifies a new corridor candidate with unresolved cells, `cert_gain` increases and the drone pivots to resolve it. The weight `w_cert = 4.0` was selected empirically and is exposed as a tunable parameter.

### 3.5 Certification Criteria

A corridor is certified when both conditions are satisfied simultaneously:

- `clearance ≥ 2.0 fine cells` — minimum distance from any hazard or inflated cell to any cell on the path.
- `coverage = 100%` — no `UNKNOWN` cells remain on the A\* path; all cells are confirmed `FREE`.

**Rationale.** Certifying a corridor with any remaining `UNKNOWN` cells is equivalent to certifying a path that may contain undetected hazards. The 100% coverage requirement ensures the drone has physically observed every cell on the output path prior to certification. The threshold is configurable (`min_coverage_ratio`) to allow relaxation where time constraints take precedence.

---

## 4. Implementation

The simulation is structured with Clean Architecture. `sim.py` is a thin CLI entry point; all logic resides under `src/`.

### Layer Structure and Import Rules

```
sim.py                               ← CLI entry point (no logic)
src/
  domain/                            ← Pure data types and invariants
  ports/                             ← Protocol definitions (interfaces)
  adapters/                          ← Port implementations
  use_cases/                         ← Business logic, no framework dependencies
  application/                       ← Tick loop orchestration
  infrastructure/                    ← Ground truth map, renderer
  experiments/                       ← Config and entry points
tests/                               ← Mirrors src/ layout
```

| Layer | May import from |
|-------|----------------|
| `domain/` | Nothing within `src/` |
| `ports/` | `domain/` only |
| `use_cases/` | `domain/`, `ports/` |
| `adapters/` | `domain/`, `ports/`, `infrastructure/` |
| `application/` | All layers |
| `experiments/` | All layers |

Map 1 (ground truth) is accessible to `infrastructure/` only. No use case reads it directly.

### Use Cases

| ID | Function | Module | Status |
|----|----------|--------|--------|
| UC1 | Observe block; update Map 3; recompute inflation | `use_cases/update_world_model.py` | Implemented |
| UC2 | A\* on Map 3; compute corridor candidate | `use_cases/compute_best_corridor.py` | Implemented |
| UC3 | Evaluate clearance and coverage; certify | `use_cases/evaluate_and_certify_corridor.py` | Implemented |
| UC4 | Reverse Dijkstra; score and rank frontiers | `use_cases/score_frontiers_corridor_aware.py` | Implemented |

### Per-Tick Loop

```
UC1  observe_block     → reveal K×K fine cells from Map 1 into Map 3
                         recompute inflation if a hazard was found
UC2  compute_corridor  → A* from start to goal on Map 3
UC3  certify           → clearance ≥ 2.0 AND coverage = 100%
                         if CERTIFIED: emit CorridorReport, halt
UC4  score_frontiers   → Reverse Dijkstra (1× per tick) over Map 3
                         score(b) = −(travel_cost + goal_cost) + w_cert × cert_gain
     move              → advance drone to highest-scoring frontier block
```

UC2 executes before UC4 so that `cert_gain` is evaluated against the latest corridor candidate within the same tick.

### Integration Benchmark

Seed 42, default configuration:

```
✅  CERTIFIED at tick 263  clearance=3.61 fine-cells  path_len=93 fine-cells
elapsed=4.64s
```

Certification at tick 263 of 600. This result is locked as the integration test in `tests/application/` and must remain stable across refactors.

---

## 5. File Index

| Path | Description |
|------|-------------|
| `sim.py` | CLI entry point — no logic |
| `src/domain/types.py` | Cell state enum, K constant, block↔fine coordinate helpers |
| `src/domain/world_model.py` | `WorldModel`: Map 2 (visited blocks) + Map 3 (detected fine cells) |
| `src/domain/drone_state.py` | `DroneState`: block and fine-cell position |
| `src/ports/ground_truth_port.py` | `GroundTruthPort` protocol: `is_hazard(fx, fy)` |
| `src/adapters/sim_ground_truth_adapter.py` | Wraps `GroundTruthMap` behind `GroundTruthPort` |
| `src/use_cases/types.py` | `CertificationResult` DTO |
| `src/use_cases/update_world_model.py` | UC1 |
| `src/use_cases/compute_best_corridor.py` | UC2 |
| `src/use_cases/evaluate_and_certify_corridor.py` | UC3 |
| `src/use_cases/score_frontiers_corridor_aware.py` | UC4 |
| `src/application/simulator.py` | Wires UC1–UC4 per tick |
| `src/infrastructure/env/ground_truth_map.py` | `GroundTruthMap` (Map 1, hidden from planner) |
| `src/infrastructure/env/map_factory.py` | Seeded random hazard placement |
| `src/infrastructure/visualization/renderer.py` | `build_truth_image` / `build_detected_image` |
| `src/experiments/config.py` | `Config` dataclass — all tunable parameters |
| `src/experiments/run_sim.py` | `run_headless`, `run_animated`, `main()` |
| `tests/` | 43 tests mirroring `src/` layout |

---

## 6. Open Items

### Immediate Next Steps

**1. Raspberry Pi Deployment and Compute Profiling**

The immediate priority is to deploy `sim.py` on the target onboard hardware (Raspberry Pi) and profile per-tick compute cost. The purpose is twofold: validate that the planning loop is feasible within the onboard resource budget, and produce concrete timing data to inform how the tick budget should be allocated across the full multi-drone system.

Metrics to collect per tick:

| Metric | Relevance |
|--------|-----------|
| A\* wall time (UC2) | Dominant cost; scales with fine grid size and UNKNOWN cell density |
| Reverse Dijkstra wall time (UC4) | Fixed O(N) per tick over the fine grid |
| Total tick wall time | Must fit within the real-time control loop period |
| Peak memory usage | Two fine grids (80×60 float32) plus A\* open set |

Profiling should cover multiple seeds and hazard densities (`n_hazards` 10–50) to capture best- and worst-case costs. If A\* wall time is the bottleneck, the primary mitigation is reducing `K` (fine cells per block side) from 4 to 2 or 3, reducing the fine grid from 4,800 to 900–2,025 cells at the cost of clearance measurement precision.

**2. Parameter Sweep**

Following the RPi profiling, run a systematic sweep to identify the configuration that minimises `time_to_certified` within the confirmed compute budget. Key parameters: `unknown_cost` (2–8), `w_cert` (1–8), `inflation_radius` (2–5), `min_coverage_ratio` (0.8–1.0). Sweep across multiple seeds and hazard densities; report `time_to_certified`, `corridor_width`, `corridor_length`.

### Deferred Work

| Item | Notes |
|------|-------|
| Probabilistic hazard fusion | `p_detect` is stored but not applied in map updates. A Bayesian update rule would replace the current binary write. |
| Localization uncertainty propagation | Inflation radius is currently fixed. It should scale with pose covariance when that data becomes available from the SLAM module. |
| Verification scheduling | A targeted re-observation pass for corridors that are near the coverage threshold but not yet certifiable. |
| Corridor-as-region width optimisation | The certified corridor is a centerline path. Explicit width optimisation would treat it as a band and maximise minimum clearance across its full extent. |

### Architectural Invariants

| Rule | Rationale |
|------|-----------|
| A\* and Dijkstra reside in `use_cases/`, not `domain/` | Domain entities are pure data structures. Algorithms in the domain layer cannot be independently tested or swapped without modifying the entities themselves. |
| Grid parameters (`block_cols`, `block_rows`, `inflation_radius`) are always read from `Config` | Hardcoded grid parameters break the parameter sweep and require source edits for map-scale changes. |
| `adapters/` contains no domain logic | Adapter code translates data formats only. Business rules in adapters become coupled to the simulation backend and cannot be unit-tested in isolation. |

---

## 7. Suggested Reading Order

1. This document
2. `README.md` — setup, run commands, configuration reference
3. `src/experiments/config.py` — all tunable parameters; review before modifying behaviour
4. `src/application/simulator.py` — tick loop; shows how UC1–UC4 are composed
5. `src/use_cases/score_frontiers_corridor_aware.py` — primary algorithmic contribution; `cert_gain` logic is here
6. `system_design.md` — full entity and use-case specification; reference when extending the module