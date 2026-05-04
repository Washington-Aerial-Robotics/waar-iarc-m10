from domain.types import (
    K, UNKNOWN, FREE, HAZARD, INFLATED,
    block_to_fine_origin, fine_cells_of_block, block_center_fine,
)


def test_constants():
    assert K == 4
    assert UNKNOWN == 0 and FREE == 1 and HAZARD == 2 and INFLATED == 3


def test_block_to_fine_origin():
    assert block_to_fine_origin(0, 0) == (0, 0)
    assert block_to_fine_origin(1, 0) == (4, 0)
    assert block_to_fine_origin(0, 1) == (0, 4)
    assert block_to_fine_origin(3, 2) == (12, 8)


def test_fine_cells_of_block_count():
    assert len(fine_cells_of_block(0, 0)) == K * K


def test_fine_cells_of_block_coverage():
    cells = set(fine_cells_of_block(1, 2))
    assert (4, 8) in cells   # origin
    assert (7, 11) in cells  # far corner
    assert len(cells) == 16


def test_block_center_fine():
    assert block_center_fine(0, 0) == (2, 2)
    assert block_center_fine(1, 0) == (6, 2)
    assert block_center_fine(0, 1) == (2, 6)
