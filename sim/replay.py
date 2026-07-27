from __future__ import annotations

import csv
from pathlib import Path

from .mines import Mine


def mines_by_timestamp(path: Path, min_confidence: float = 0.1) -> list[tuple[float, Mine]]:
    """Rows sorted by time; first time each tag_id meets confidence threshold."""
    rows: list[tuple[float, Mine]] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                conf = float(row.get("confidence") or 0)
                if conf < min_confidence:
                    continue
                ts = float(row["timestamp"])
                tag_id = int(row["tag_id"])
                wx = float(row["world_x"])
                wy = float(row["world_y"])
            except (KeyError, TypeError, ValueError):
                continue
            rows.append((ts, Mine(tag_id=tag_id, world_x=wx, world_y=wy, confidence=conf)))

    rows.sort(key=lambda r: r[0])
    seen: set[int] = set()
    events: list[tuple[float, Mine]] = []
    for ts, mine in rows:
        if mine.tag_id in seen:
            continue
        seen.add(mine.tag_id)
        events.append((ts, mine))
    return events
