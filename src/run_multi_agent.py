from __future__ import annotations

from application.simulator import Simulator

from multi_agent.coordinator import (
    MultiAgentCoordinator,
)

from domain.world_model import WorldModel
from domain.drone_state import DroneState

from infrastructure.ground_truth import (
    GroundTruth,
)

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

gt = GroundTruth.from_random(
    fine_cols=BLOCK_COLS * 4,
    fine_rows=BLOCK_ROWS * 4,
    n_hazards=30,
    seed=42,
)

# ---------------------------------------------------------
# Create independent simulators
# ---------------------------------------------------------

simulators = []

spawn_blocks = [
    (0, 0),
    (0, 0),
    (0, 0),
    (0, 0),
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

MIN_GLOBAL_COVERAGE = 0.65   # require 65% of the arena scanned before certifying

coordinator = MultiAgentCoordinator(
    simulators,
    min_global_coverage=MIN_GLOBAL_COVERAGE,
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

        winner_id = coordinator.winner()
        winning_sim = next(s for s in simulators if s.drone_id == winner_id)

        print("Corridor start:", winning_sim.mission.corridor[0])
        print("Corridor end:  ", winning_sim.mission.corridor[-1])
        print("start_line range:", winning_sim.world.start_line[0], "to", winning_sim.world.start_line[-1])
        print("goal_line range: ", winning_sim.world.goal_line[0], "to", winning_sim.world.goal_line[-1])

        break

else:

    print()
    print(
        "Mission failed to certify "
        "within max ticks."
    )