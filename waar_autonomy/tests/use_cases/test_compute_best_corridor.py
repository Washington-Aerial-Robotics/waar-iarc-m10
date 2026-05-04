from domain.types import FREE, HAZARD, INFLATED
from domain.world_model import WorldModel
from use_cases.compute_best_corridor import compute_best_corridor


def _all_free(wm: WorldModel) -> None:
    wm.detected[:] = FREE


def test_finds_path_when_all_free():
    wm = WorldModel(6, 4)
    _all_free(wm)
    path = compute_best_corridor(wm, unknown_cost=4.0)
    assert path is not None
    assert path[0] == wm.start_fine
    assert path[-1] == wm.goal_fine


def test_returns_none_when_goal_blocked():
    wm = WorldModel(6, 4)
    _all_free(wm)
    gx, gy = wm.goal_fine
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < wm.fine_cols and 0 <= ny < wm.fine_rows:
                wm.detected[nx, ny] = HAZARD
    path = compute_best_corridor(wm, unknown_cost=4.0)
    assert path is None


def test_path_avoids_hazards():
    wm = WorldModel(6, 4)
    _all_free(wm)
    # Block a vertical strip of HAZARD cells through the middle
    mid = wm.fine_cols // 2
    wm.detected[mid, :] = HAZARD
    path = compute_best_corridor(wm, unknown_cost=4.0)
    if path is not None:
        assert all(wm.detected[x, y] != HAZARD for x, y in path)


def test_path_avoids_inflated():
    wm = WorldModel(6, 4)
    _all_free(wm)
    mid = wm.fine_cols // 2
    wm.detected[mid, :] = INFLATED
    path = compute_best_corridor(wm, unknown_cost=4.0)
    if path is not None:
        assert all(wm.detected[x, y] != INFLATED for x, y in path)
