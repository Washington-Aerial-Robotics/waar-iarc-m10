from domain.types import FREE, HAZARD, UNKNOWN
from domain.world_model import WorldModel
from use_cases.evaluate_and_certify_corridor import (
    evaluate_corridor, corridor_clearance, corridor_coverage,
)


def _make_world_with_path() -> tuple:
    wm = WorldModel(6, 4)
    wm.detected[:] = FREE
    path = [(x, wm.fine_rows // 2) for x in range(wm.fine_cols)]
    return wm, path


def test_certifies_when_thresholds_met():
    wm, path = _make_world_with_path()
    result = evaluate_corridor(path, wm, min_clearance_cells=0.0, min_coverage_ratio=1.0)
    assert result.certified
    assert result.coverage == 1.0


def test_not_certified_when_unknown_on_path():
    wm, path = _make_world_with_path()
    px, py = path[5]
    wm.detected[px, py] = UNKNOWN
    result = evaluate_corridor(path, wm, min_clearance_cells=0.0, min_coverage_ratio=1.0)
    assert not result.certified
    assert result.coverage < 1.0


def test_not_certified_when_hazard_too_close():
    wm, path = _make_world_with_path()
    px, py = path[5]
    wm.detected[px, py + 1] = HAZARD
    result = evaluate_corridor(path, wm, min_clearance_cells=2.0, min_coverage_ratio=0.0)
    assert not result.certified


def test_clearance_inf_when_no_hazards():
    wm, path = _make_world_with_path()
    clr = corridor_clearance(path, wm)
    assert clr == float("inf")


def test_empty_path_returns_not_certified():
    wm = WorldModel(6, 4)
    result = evaluate_corridor(None, wm, 2.0, 1.0)
    assert not result.certified
    assert result.clearance == 0.0
    assert result.coverage  == 0.0
