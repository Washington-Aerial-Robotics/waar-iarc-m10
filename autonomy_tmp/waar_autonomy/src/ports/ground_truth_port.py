from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class GroundTruthPort(Protocol):
    """Read-only view of the hidden ground-truth hazard map (map1)."""

    def is_hazard(self, fx: int, fy: int) -> bool:
        """Return True if fine cell (fx, fy) contains a hazard."""
        ...
