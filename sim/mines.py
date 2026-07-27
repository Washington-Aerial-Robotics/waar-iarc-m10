from __future__ import annotations

import csv
import json
import random
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Mine:
    tag_id: int
    world_x: float
    world_y: float
    confidence: float = 1.0


def load_mines_from_csv(path: Path, min_confidence: float = 0.1) -> list[Mine]:
    """Aggregate detections by tag_id (highest-confidence row wins)."""
    best: dict[int, Mine] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                conf = float(row.get("confidence") or 0)
                if conf < min_confidence:
                    continue
                tag_id = int(row["tag_id"])
                wx = float(row["world_x"])
                wy = float(row["world_y"])
            except (KeyError, TypeError, ValueError):
                continue
            mine = Mine(tag_id=tag_id, world_x=wx, world_y=wy, confidence=conf)
            if tag_id not in best or conf > best[tag_id].confidence:
                best[tag_id] = mine
    return sorted(best.values(), key=lambda m: m.tag_id)


def load_mines_from_json(path: Path) -> list[Mine]:
    """Load fused mines from obstacle-style export or a simple list of dicts."""
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict) and "mines" in data:
        records = data["mines"]
    elif isinstance(data, list):
        records = data
    else:
        return []

    mines: list[Mine] = []
    for rec in records:
        if "tag_id" not in rec:
            continue
        mines.append(
            Mine(
                tag_id=int(rec["tag_id"]),
                world_x=float(rec.get("world_x", rec.get("x", 0))),
                world_y=float(rec.get("world_y", rec.get("y", 0))),
                confidence=float(rec.get("confidence", 1.0)),
            )
        )
    return mines


def generate_random_mines(
    count: int,
    field_x_m: float,
    field_y_m: float,
    margin_m: float,
    seed: int,
) -> list[Mine]:
    rng = random.Random(seed)
    mines: list[Mine] = []
    for tag_id in range(count):
        wx = rng.uniform(margin_m, field_x_m - margin_m)
        wy = rng.uniform(margin_m, field_y_m - margin_m)
        mines.append(Mine(tag_id=tag_id, world_x=wx, world_y=wy, confidence=1.0))
    return mines
