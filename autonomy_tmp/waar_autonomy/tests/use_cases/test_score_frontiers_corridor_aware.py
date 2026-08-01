from domain.types import FREE, UNKNOWN
from domain.world_model import WorldModel
from domain.drone_state import DroneState
from use_cases.score_frontiers_corridor_aware import best_frontier, _frontier_blocks


def _setup(block_cols=8, block_rows=6):
    wm = WorldModel(block_cols, block_rows)
    wm.detected[:] = FREE
    drone = DroneState(block_cols, block_rows)
    # visit drone's starting block so frontier detection works
    wm.visited[drone.block[0], drone.block[1]] = True
    return wm, drone


def test_frontier_blocks_found_after_one_visit():
    wm, drone = _setup()
    fronts = _frontier_blocks(wm)
    assert len(fronts) > 0
    # no frontier should be a visited block
    for b in fronts:
        assert not wm.visited[b[0], b[1]]


def test_best_frontier_returns_a_block():
    wm, drone = _setup()
    nxt = best_frontier(wm, drone, None, unknown_cost=4.0, w_cert=4.0)
    assert nxt is not None


def test_best_frontier_not_already_visited():
    wm, drone = _setup()
    nxt = best_frontier(wm, drone, None, unknown_cost=4.0, w_cert=4.0)
    assert not wm.visited[nxt[0], nxt[1]]


def test_no_frontiers_returns_none():
    wm = WorldModel(3, 3)
    wm.detected[:] = FREE
    wm.visited[:] = True   # everything visited → no frontiers
    drone = DroneState(3, 3)
    result = best_frontier(wm, drone, None, unknown_cost=4.0, w_cert=4.0)
    assert result is None
