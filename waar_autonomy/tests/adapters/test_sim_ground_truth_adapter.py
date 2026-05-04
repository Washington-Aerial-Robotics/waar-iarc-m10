from infrastructure.env.ground_truth_map import GroundTruthMap
from adapters.sim_ground_truth_adapter import SimGroundTruthAdapter
from ports.ground_truth_port import GroundTruthPort


def test_satisfies_protocol():
    gt = GroundTruthMap(20, 16)
    adapter = SimGroundTruthAdapter(gt)
    assert isinstance(adapter, GroundTruthPort)


def test_delegates_is_hazard():
    gt = GroundTruthMap(20, 16)
    gt.data[3, 7] = True
    adapter = SimGroundTruthAdapter(gt)
    assert adapter.is_hazard(3, 7)
    assert not adapter.is_hazard(3, 8)
