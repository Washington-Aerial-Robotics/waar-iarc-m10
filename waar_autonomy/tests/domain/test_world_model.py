import numpy as np
from domain.types import K, UNKNOWN, FREE
from domain.world_model import WorldModel


def test_shape():
    wm = WorldModel(10, 8)
    assert wm.visited.shape  == (10, 8)
    assert wm.detected.shape == (40, 32)
    assert wm.fine_cols == 40
    assert wm.fine_rows == 32


def test_initial_state_unknown():
    wm = WorldModel(5, 4)
    # most cells start as UNKNOWN (start/goal pre-marked FREE)
    assert wm.detected[0, 0] == UNKNOWN


def test_start_goal_premarked_free():
    wm = WorldModel(10, 8)
    sx, sy = wm.start_fine
    gx, gy = wm.goal_fine
    assert wm.detected[sx, sy] == FREE
    assert wm.detected[gx, gy] == FREE


def test_start_goal_positions():
    wm = WorldModel(10, 8)
    assert wm.start_fine == (K, wm.fine_rows - K - 1)
    assert wm.goal_fine  == (wm.fine_cols - K - 1, K)


def test_visited_initially_empty():
    wm = WorldModel(10, 8)
    assert not wm.visited.any()
