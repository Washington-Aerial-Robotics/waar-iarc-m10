from domain.world_model import WorldModel
from domain.drone_state import DroneState
from domain.types import FREE
from infrastructure.env.map_factory import build_ground_truth_map
from adapters.sim_ground_truth_adapter import SimGroundTruthAdapter
from application.simulator import Simulator, MissionState


def _make_simulator(block_cols=10, block_rows=8, n_hazards=5, seed=42):
    wm    = WorldModel(block_cols, block_rows)
    drone = DroneState(block_cols, block_rows)
    gt    = SimGroundTruthAdapter(build_ground_truth_map(block_cols, block_rows, n_hazards, seed))
    return Simulator(
        world=wm, drone=drone, gt=gt,
        inflation_radius=3.5, unknown_cost=4.0,
        min_clearance_cells=2.0, min_coverage_ratio=1.0,
        w_cert=4.0,
    )


def test_initial_state():
    sim = _make_simulator()
    assert sim.mission.tick == 0
    assert not sim.mission.certified
    assert sim.mission.corridor is None


def test_start_block_observed_on_init():
    sim = _make_simulator()
    bx, by = sim.drone.block
    assert sim.world.visited[bx, by]


def test_tick_increments_counter():
    sim = _make_simulator()
    sim.tick()
    assert sim.mission.tick == 1


def test_tick_moves_drone():
    sim = _make_simulator()
    initial_block = sim.drone.block
    sim.tick()
    assert sim.drone.block != initial_block


def test_simulation_certifies_within_budget():
    """Full headless run with seed 42; must certify within 600 ticks."""
    from domain.world_model import WorldModel
    from domain.drone_state import DroneState
    from infrastructure.env.map_factory import build_ground_truth_map
    from adapters.sim_ground_truth_adapter import SimGroundTruthAdapter

    wm    = WorldModel(20, 15)
    drone = DroneState(20, 15)
    gt    = SimGroundTruthAdapter(build_ground_truth_map(20, 15, 25, seed=42))
    sim   = Simulator(
        world=wm, drone=drone, gt=gt,
        inflation_radius=3.5, unknown_cost=4.0,
        min_clearance_cells=2.0, min_coverage_ratio=1.0,
        w_cert=4.0,
    )

    done = False
    while sim.mission.tick < 600 and not done:
        done = sim.tick()

    assert sim.mission.certified, "expected certification within 600 ticks"
    assert sim.mission.cert_tick == 263        # must match reference run
    assert abs(sim.mission.cert_clearance - 3.61) < 0.01
    assert len(sim.mission.corridor) == 93
