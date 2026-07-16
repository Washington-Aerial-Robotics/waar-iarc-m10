"""
visualize_multi_agent.py
------------------------
Drop this file into waar_autonomy/src/ alongside run_multi_agent.py.
Run with:
    python visualize_multi_agent.py

Renders:
  - Arena fine grid (80x60) coloured by cell state (UNKNOWN/FREE/HAZARD/INFLATED)
  - True hazard positions (red X markers)
  - Each drone's path history as a coloured trail
  - Final drone positions as labelled markers
  - Certification tick annotation
"""

from __future__ import annotations

import matplotlib
matplotlib.use("Agg")  # headless — swap to "TkAgg" or "Qt5Agg" if you want a live window
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patheffects as pe
import numpy as np
from typing import Optional

# ── project imports ────────────────────────────────────────────────────────────
from application.simulator import Simulator
from multi_agent.coordinator import MultiAgentCoordinator
from domain.world_model import WorldModel
from domain.drone_state import DroneState
from domain.types import UNKNOWN, FREE, HAZARD, INFLATED
from infrastructure.ground_truth import GroundTruth

# ── configuration ──────────────────────────────────────────────────────────────
NUM_DRONES        = 4
BLOCK_COLS        = 20
BLOCK_ROWS        = 15
K                 = 4          # fine cells per block side
FINE_COLS         = BLOCK_COLS * K   # 80
FINE_ROWS         = BLOCK_ROWS * K   # 60
INFLATION_RADIUS  = 2.0
UNKNOWN_COST      = 3.0
MIN_CLEARANCE     = 2.0
MIN_COVERAGE      = 0.95
W_CERT            = 5.0
MAX_TICKS         = 600
N_HAZARDS         = 60
SEED              = 42
MIN_GLOBAL_COVERAGE = 0   # require this fraction of the arena scanned before certifying
start_x = 0
start_y = 10

DRONE_COLORS = ["#00C8FF", "#FF6B35", "#A8FF3E", "#FFD700"]
SPAWN_BLOCKS = [(start_x, start_y), (start_x, start_y), (start_x, start_y), (start_x, start_y)]

# ── cell-state palette ─────────────────────────────────────────────────────────
CELL_COLORS = {
    UNKNOWN:  "#1a1a2e",   # deep navy
    FREE:     "#16213e",   # explored dark blue
    HAZARD:   "#e94560",   # vivid red
    INFLATED: "#533483",   # purple buffer
}


def run_simulation() -> tuple[
    list[Simulator],
    list[list[tuple[int, int]]],   # per-drone block path history
    GroundTruth,
    Optional[str],
    int,
]:
    gt = GroundTruth.from_random(
        fine_cols=FINE_COLS,
        fine_rows=FINE_ROWS,
        n_hazards=N_HAZARDS,
        seed=SEED,
    )

    simulators = []
    histories: list[list[tuple[int, int]]] = []

    for i in range(NUM_DRONES):
        world = WorldModel(block_cols=BLOCK_COLS, block_rows=BLOCK_ROWS)
        drone = DroneState(block=SPAWN_BLOCKS[i])
        sim = Simulator(
            drone_id=f"D{i}",
            world=world,
            drone=drone,
            gt=gt,
            inflation_radius=INFLATION_RADIUS,
            unknown_cost=UNKNOWN_COST,
            min_clearance_cells=MIN_CLEARANCE,
            min_coverage_ratio=MIN_COVERAGE,
            w_cert=W_CERT,
        )
        simulators.append(sim)
        histories.append([SPAWN_BLOCKS[i]])

    coordinator = MultiAgentCoordinator(simulators, min_global_coverage=MIN_GLOBAL_COVERAGE)

    cert_tick = 0
    for t in range(1, MAX_TICKS + 1):
        done = coordinator.tick()
        for i, sim in enumerate(simulators):
            histories[i].append(sim.drone.block)
        if done:
            cert_tick = t
            break

    return simulators, histories, gt, coordinator.winner(), cert_tick


def block_to_fine_center(bx: int, by: int) -> tuple[float, float]:
    """Convert block coords to fine-grid pixel centre."""
    return bx * K + K / 2, by * K + K / 2


def render(
    simulators: list[Simulator],
    histories: list[list[tuple[int, int]]],
    gt: GroundTruth,
    winner: Optional[str],
    cert_tick: int,
    save_path: str = "multi_agent_arena.png",
) -> None:

    # ── build cell-state image from the UNION of all drone detected maps ───────
    # Use the last simulator's world as representative (or merge all)
    union_detected = np.zeros((FINE_COLS, FINE_ROWS), dtype=np.int8)
    for sim in simulators:
        d = sim.world.detected
        # FREE and HAZARD/INFLATED override UNKNOWN
        mask_free     = d == FREE
        mask_hazard   = d == HAZARD
        mask_inflated = d == INFLATED
        union_detected[mask_free]     = FREE
        union_detected[mask_hazard]   = HAZARD
        union_detected[mask_inflated] = INFLATED

    # Build RGB image  (transpose: detected is [col, row], imshow wants [row, col])
    img = np.zeros((FINE_ROWS, FINE_COLS, 3))
    for state, hex_color in CELL_COLORS.items():
        r, g, b = tuple(int(hex_color.lstrip("#")[i:i+2], 16) / 255 for i in (0, 2, 4))
        mask = (union_detected.T == state)
        img[mask] = (r, g, b)

    # ── figure setup ───────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(16, 10), facecolor="#0d0d1a")
    ax.set_facecolor("#0d0d1a")

    ax.imshow(
        img,
        origin="lower",
        extent=[0, FINE_COLS, 0, FINE_ROWS],
        interpolation="nearest",
        zorder=0,
    )

    # ── block grid lines (subtle) ──────────────────────────────────────────────
    for x in range(0, FINE_COLS + 1, K):
        ax.axvline(x, color="#ffffff08", linewidth=0.4, zorder=1)
    for y in range(0, FINE_ROWS + 1, K):
        ax.axhline(y, color="#ffffff08", linewidth=0.4, zorder=1)

    # ── true hazard positions ──────────────────────────────────────────────────
    hxs, hys = zip(*gt.get_hazard_positions()) if gt.get_hazard_positions() else ([], [])
    ax.scatter(
        [x + 0.5 for x in hxs],
        [y + 0.5 for y in hys],
        marker="x",
        s=120,
        linewidths=2.5,
        color="#ff2d55",
        zorder=5,
        label="True hazard",
    )
    # glow ring around each hazard
    ax.scatter(
        [x + 0.5 for x in hxs],
        [y + 0.5 for y in hys],
        marker="o",
        s=400,
        linewidths=1,
        facecolors="none",
        edgecolors="#ff2d5550",
        zorder=4,
    )

    # ── drone paths ────────────────────────────────────────────────────────────
    for i, (sim, history, color) in enumerate(zip(simulators, histories, DRONE_COLORS)):
        fx = [block_to_fine_center(bx, by)[0] for bx, by in history]
        fy = [block_to_fine_center(bx, by)[1] for bx, by in history]

        # path trail
        ax.plot(
            fx, fy,
            color=color,
            linewidth=1.5,
            alpha=0.6,
            zorder=6,
            solid_capstyle="round",
        )
        # start marker
        ax.scatter(
            fx[0], fy[0],
            marker="s", s=80, color=color, zorder=8, alpha=0.5,
        )
        # end marker + label
        ax.scatter(
            fx[-1], fy[-1],
            marker="o", s=180, color=color, zorder=9,
            edgecolors="white", linewidths=1.5,
        )
        label = f"{sim.drone_id}{'★' if sim.drone_id == winner else ''}"
        ax.text(
            fx[-1] + 1.2, fy[-1] + 1.2,
            label,
            color=color,
            fontsize=9,
            fontweight="bold",
            zorder=10,
            path_effects=[pe.withStroke(linewidth=2, foreground="#0d0d1a")],
        )

    # ── certified corridor ────────────────────────────────────────────────────
    winning_sim = next((s for s in simulators if s.drone_id == winner), None)
    if winning_sim and winning_sim.mission.corridor:
        cx = [fc + 0.5 for fc, _ in winning_sim.mission.corridor]
        cy = [fr + 0.5 for _, fr in winning_sim.mission.corridor]
        ax.plot(
            cx, cy,
            color="#ffffff",
            linewidth=2,
            alpha=0.35,
            linestyle="--",
            zorder=7,
            label="Certified corridor",
        )

    # ── legend & labels ───────────────────────────────────────────────────────
    legend_patches = [
        mpatches.Patch(color=CELL_COLORS[UNKNOWN],  label="Unknown"),
        mpatches.Patch(color=CELL_COLORS[FREE],     label="Free (explored)"),
        mpatches.Patch(color=CELL_COLORS[HAZARD],   label="Hazard (detected)"),
        mpatches.Patch(color=CELL_COLORS[INFLATED], label="Inflated buffer"),
        mpatches.Patch(color="#ff2d55",             label="True hazard (ground truth)"),
    ]
    drone_patches = [
        mpatches.Patch(color=c, label=f"Drone {i} path")
        for i, c in enumerate(DRONE_COLORS[:NUM_DRONES])
    ]
    ax.legend(
        handles=legend_patches + drone_patches,
        loc="upper right",
        fontsize=8,
        framealpha=0.3,
        facecolor="#0d0d1a",
        edgecolor="#ffffff30",
        labelcolor="white",
    )

    cert_text = (
        f"CORRIDOR CERTIFIED  ·  tick {cert_tick}  ·  winner {winner} · minimum global coverage constraint {MIN_GLOBAL_COVERAGE}   "
        if winner else f"Not certified after {MAX_TICKS} ticks"
    )
    ax.set_title(
        cert_text,
        color="white",
        fontsize=13,
        fontweight="bold",
        pad=12,
    )
    ax.set_xlabel("Fine grid X  (1 unit = 0.25 m)", color="#aaaaaa", fontsize=9)
    ax.set_ylabel("Fine grid Y  (1 unit = 0.25 m)", color="#aaaaaa", fontsize=9)
    ax.tick_params(colors="#555555")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333333")

    ax.set_xlim(0, FINE_COLS)
    ax.set_ylim(0, FINE_ROWS)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"Saved → {save_path}")


if __name__ == "__main__":
    print("Running simulation…")
    simulators, histories, gt, winner, cert_tick = run_simulation()
    print(f"Certified at tick {cert_tick} by {winner}")
    render(simulators, histories, gt, winner, cert_tick)