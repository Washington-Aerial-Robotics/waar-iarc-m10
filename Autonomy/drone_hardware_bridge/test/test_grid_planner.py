import pytest

from drone_hardware_bridge.grid_planner import Grid, GridPlanner, lawnmower_waypoints


def grid(width, height, data=None, resolution=1.0):
    return Grid(width, height, resolution, 0.0, 0.0, 0.0, tuple(data or [0] * (width * height)))


def test_astar_detours_around_obstacle():
    data = [0] * 25
    data[2 * 5 + 2] = 100
    planner = GridPlanner(grid(5, 5, data), inflation_radius_m=0.0)
    path = planner.plan((0.5, 2.5), (4.5, 2.5))
    assert path
    assert (2, 2) not in [planner.grid.world_to_cell(point) for point in path]


def test_inflation_blocks_neighbor_cell():
    data = [0] * 25
    data[2 * 5 + 2] = 100
    planner = GridPlanner(grid(5, 5, data), inflation_radius_m=1.0)
    assert not planner.is_free((2, 1))
    assert planner.is_free((0, 0))


def test_unknown_rejected_by_default_and_can_be_configured():
    data = [0, -1, 0]
    assert GridPlanner(grid(3, 1, data), inflation_radius_m=0).plan((0.5, 0.5), (2.5, 0.5)) == []
    assert GridPlanner(
        grid(3, 1, data), unknown_is_blocked=False, inflation_radius_m=0
    ).plan((0.5, 0.5), (2.5, 0.5))


def test_no_path_and_out_of_bounds_fail_closed():
    data = [0, 100, 0, 0, 100, 0, 0, 100, 0]
    planner = GridPlanner(grid(3, 3, data), inflation_radius_m=0)
    assert planner.plan((0.5, 0.5), (2.5, 0.5)) == []
    assert planner.plan((-0.1, 0.5), (0.5, 0.5)) == []


def test_lawnmower_is_deterministic_and_bounded():
    path = lawnmower_waypoints(1.0, 5.0, 2.0, 6.0, 2.0)
    assert path == [
        (1.0, 2.0), (5.0, 2.0), (5.0, 4.0),
        (1.0, 4.0), (1.0, 6.0), (5.0, 6.0),
    ]
    assert all(1.0 <= x <= 5.0 and 2.0 <= y <= 6.0 for x, y in path)
