import numpy as np
from domain.types import K, UNKNOWN, FREE, HAZARD, INFLATED, fine_cells_of_block
from domain.world_model import WorldModel
from use_cases.update_world_model import observe_block


class _NoHazard:
    def is_hazard(self, fx, fy): return False


class _HazardAt:
    def __init__(self, *cells): self._cells = set(cells)
    def is_hazard(self, fx, fy): return (fx, fy) in self._cells


def test_observe_block_marks_visited():
    wm = WorldModel(5, 4)
    observe_block(wm, _NoHazard(), 1, 2, 0.0)
    assert wm.visited[1, 2]


def test_observe_block_marks_free():
    wm = WorldModel(5, 4)
    observe_block(wm, _NoHazard(), 1, 2, 0.0)
    for fx, fy in fine_cells_of_block(1, 2):
        assert wm.detected[fx, fy] == FREE


def test_observe_block_marks_hazard():
    wm = WorldModel(5, 4)
    observe_block(wm, _HazardAt((4, 8)), 1, 2, 0.0)
    assert wm.detected[4, 8] == HAZARD


def test_inflation_applied():
    wm = WorldModel(10, 8)
    observe_block(wm, _HazardAt((20, 16)), 5, 4, 2.0)
    # neighbours within radius 2 should be INFLATED (unless HAZARD itself)
    assert wm.detected[20, 16] == HAZARD
    assert wm.detected[20, 17] == INFLATED


def test_inflation_does_not_overwrite_hazard():
    wm = WorldModel(10, 8)
    observe_block(wm, _HazardAt((20, 16), (20, 17)), 5, 4, 2.0)
    assert wm.detected[20, 16] == HAZARD
    assert wm.detected[20, 17] == HAZARD


def test_observe_is_idempotent():
    wm = WorldModel(5, 4)
    observe_block(wm, _NoHazard(), 1, 2, 0.0)
    observe_block(wm, _NoHazard(), 1, 2, 0.0)
    for fx, fy in fine_cells_of_block(1, 2):
        assert wm.detected[fx, fy] == FREE
