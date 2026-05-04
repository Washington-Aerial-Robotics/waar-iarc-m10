from domain.types import block_center_fine
from domain.drone_state import DroneState


def test_initial_position():
    d = DroneState(10, 8)
    assert d.block == (0, 6)   # (0, block_rows - 2)
    assert d.fine  == block_center_fine(0, 6)


def test_move_to():
    d = DroneState(10, 8)
    d.move_to(3, 5)
    assert d.block == (3, 5)
    assert d.fine  == block_center_fine(3, 5)
