from __future__ import annotations

from application.simulator import Simulator

from multi_agent.coordinator import (
    MultiAgentCoordinator,
)

from domain.world_model import WorldModel
from domain.drone_state import DroneState

from adapters.sim_ground_truth_adapter import SimGroundTruthAdapter
from infrastructure.env.map_factory import build_ground_truth_map

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------

NUM_DRONES = 4

BLOCK_COLS = 20
BLOCK_ROWS = 15

INFLATION_RADIUS = 2.0

UNKNOWN_COST = 3.0

MIN_CLEARANCE = 2.0

MIN_COVERAGE = 0.95

W_CERT = 5.0

# ---------------------------------------------------------
# Ground truth environment
# ---------------------------------------------------------

gt_map = build_ground_truth_map(
    BLOCK_COLS,
    BLOCK_ROWS,
    n_hazards=10,
    seed=42,
)
gt = SimGroundTruthAdapter(gt_map)

# ---------------------------------------------------------
# Create independent simulators
# ---------------------------------------------------------

simulators = []

spawn_blocks = [
    (0, 0),
    (0, 14),
    (19, 0),
    (19, 14),
]

for i in range(NUM_DRONES):

    world = WorldModel(
        block_cols=BLOCK_COLS,
        block_rows=BLOCK_ROWS,
    )

    drone = DroneState(
        block=spawn_blocks[i],
    )

    sim = Simulator(
        drone_id=f"D{i}",

        world=world,

        drone=drone,

        gt=gt,

        inflation_radius=INFLATION_RADIUS,

        unknown_cost=UNKNOWN_COST,

        min_clearance_cells=MIN_CLEARANCE,

        min_coverage_ratio=MIN_COVERAGE,

        w_cert=W_CERT,
    )

    simulators.append(sim)

# ---------------------------------------------------------
# Create coordinator
# ---------------------------------------------------------

coordinator = MultiAgentCoordinator(
    simulators,
)

# ---------------------------------------------------------
# Main simulation loop
# ---------------------------------------------------------

MAX_TICKS = 1000

for _ in range(MAX_TICKS):

    done = coordinator.tick()

    positions = (
        coordinator.drone_positions()
    )

    print(
        f"Tick {coordinator.state.tick}"
    )

    for drone_id, pos in positions.items():

        print(
            f"  {drone_id}: {pos}"
        )

    if done:

        print()
        print(
            "MISSION CERTIFIED"
        )

        print(
            "Winning drone:",
            coordinator.winner(),
        )

        break

else:

    print()
    print(
        "Mission failed to certify "
        "within max ticks."
    )