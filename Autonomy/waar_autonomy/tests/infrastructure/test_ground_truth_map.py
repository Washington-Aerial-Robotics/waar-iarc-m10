import numpy as np
from infrastructure.env.ground_truth_map import GroundTruthMap
from infrastructure.env.map_factory import build_ground_truth_map


def test_initial_empty():
    gt = GroundTruthMap(20, 16)
    assert not gt.data.any()


def test_is_hazard():
    gt = GroundTruthMap(20, 16)
    gt.data[5, 5] = True
    assert gt.is_hazard(5, 5)
    assert not gt.is_hazard(5, 6)


def test_build_deterministic():
    gt1 = build_ground_truth_map(10, 8, 5, seed=42)
    gt2 = build_ground_truth_map(10, 8, 5, seed=42)
    assert np.array_equal(gt1.data, gt2.data)


def test_build_hazard_count():
    gt = build_ground_truth_map(10, 8, 5, seed=42)
    assert gt.data.sum() == 5


def test_build_different_seeds_differ():
    gt1 = build_ground_truth_map(10, 8, 10, seed=1)
    gt2 = build_ground_truth_map(10, 8, 10, seed=2)
    assert not np.array_equal(gt1.data, gt2.data)
