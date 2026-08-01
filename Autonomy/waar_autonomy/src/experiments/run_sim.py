"""Experiment entry point for the single-drone corridor simulator.

Usage (from project root with PYTHONPATH=./src):
    python -m experiments.run_sim                  # animated
    python -m experiments.run_sim --seed 7
    python -m experiments.run_sim --no-anim        # headless
    python -m experiments.run_sim --delay 0.02
    python -m experiments.run_sim --hazards 40
"""
from __future__ import annotations

import argparse
import time

from domain.world_model import WorldModel
from domain.drone_state import DroneState
from experiments.config import Config
from infrastructure.env.map_factory import build_ground_truth_map
from adapters.sim_ground_truth_adapter import SimGroundTruthAdapter
from application.simulator import Simulator


def _make_simulator(cfg: Config, seed: int) -> tuple[Simulator, object]:
    wm    = WorldModel(cfg.block_cols, cfg.block_rows)
    drone = DroneState(cfg.block_cols, cfg.block_rows)
    gt_map = build_ground_truth_map(cfg.block_cols, cfg.block_rows, cfg.n_hazards, seed)
    gt    = SimGroundTruthAdapter(gt_map)
    sim   = Simulator(
        world=wm, drone=drone, gt=gt,
        inflation_radius=cfg.inflation_radius,
        unknown_cost=cfg.unknown_cost,
        min_clearance_cells=cfg.min_clearance_cells,
        min_coverage_ratio=cfg.min_coverage_ratio,
        w_cert=cfg.w_cert,
    )
    return sim, gt_map


def run_headless(sim: Simulator, cfg: Config) -> None:
    done = False
    t0   = time.time()
    while sim.mission.tick < cfg.max_ticks and not done:
        done = sim.tick()
        if sim.mission.tick % 50 == 0 or done:
            clr = sim.current_clearance()
            cov = sim.current_coverage()
            print(
                f"  tick={sim.mission.tick:4d}  "
                f"corridor={'yes' if sim.mission.corridor else 'no '}  "
                f"clearance={clr:.2f}  coverage={cov*100:.0f}%"
            )
    print(f"elapsed={time.time()-t0:.2f}s")
    _print_result(sim)


def run_animated(sim: Simulator, gt_map, cfg: Config) -> None:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from infrastructure.visualization.renderer import build_truth_image, build_detected_image
    from domain.types import UNKNOWN, FREE, HAZARD, INFLATED
    from infrastructure.visualization.renderer import _COLOURS

    fig, (ax_truth, ax_map) = plt.subplots(
        1, 2, figsize=(16, 7), gridspec_kw={"width_ratios": [1, 1]}
    )
    fig.patch.set_facecolor("#0d0d12")
    for ax in (ax_truth, ax_map):
        ax.set_facecolor("#0d0d12")

    truth_h    = ax_truth.imshow(
        build_truth_image(sim.world, gt_map, sim.mission.corridor, sim.drone.fine),
        interpolation="nearest", aspect="equal",
    )
    detected_h = ax_map.imshow(
        build_detected_image(sim.world, sim.mission.corridor, sim.drone.fine),
        interpolation="nearest", aspect="equal",
    )
    ax_truth.axis("off")
    ax_map.axis("off")
    ax_truth.set_title("Map 1 — Ground Truth", color="white", fontsize=11, pad=8)
    ax_map.set_title("Map 3 — Detected", color="white", fontsize=11, pad=8)

    legend = [
        mpatches.Patch(color=_COLOURS[UNKNOWN],  label="Unknown"),
        mpatches.Patch(color=_COLOURS[FREE],      label="Free (detected)"),
        mpatches.Patch(color=_COLOURS[HAZARD],    label="Hazard"),
        mpatches.Patch(color=_COLOURS[INFLATED],  label="Inflated zone"),
        mpatches.Patch(color=[0.0, 1.0, 0.55],   label="Best corridor"),
        mpatches.Patch(color=[1.0, 1.0, 0.00],   label="Drone"),
        mpatches.Patch(color=[0.2, 0.9, 0.2],    label="Start"),
        mpatches.Patch(color=[0.2, 0.5, 1.0],    label="Goal"),
    ]
    ax_map.legend(handles=legend, loc="lower left", fontsize=7,
                  facecolor="#1a1a24", labelcolor="white", framealpha=0.85)

    status_txt = ax_map.text(
        0.01, 0.99, "", transform=ax_map.transAxes,
        color="white", fontsize=9, va="top",
        bbox=dict(boxstyle="round", facecolor="#1a1a24", alpha=0.8),
    )

    plt.ion()
    plt.tight_layout(pad=1.5)
    plt.show()

    done = False
    while sim.mission.tick < cfg.max_ticks and not done:
        done = sim.tick()

        clr = sim.current_clearance()
        cov = sim.current_coverage()

        truth_h.set_data(
            build_truth_image(sim.world, gt_map, sim.mission.corridor, sim.drone.fine)
        )
        detected_h.set_data(
            build_detected_image(sim.world, sim.mission.corridor, sim.drone.fine)
        )

        status = (
            f"Tick {sim.mission.tick}  |  "
            f"Corridor: {'found' if sim.mission.corridor else '—'}  |  "
            f"Clearance: {clr:.1f}  |  Coverage: {cov*100:.0f}%"
        )
        if done:
            status += "  ✅ CERTIFIED"
        status_txt.set_text(status)
        plt.pause(cfg.tick_delay)

    _print_result(sim)
    plt.ioff()
    plt.show()


def _print_result(sim: Simulator) -> None:
    if sim.mission.certified:
        print(
            f"\n✅  CERTIFIED at tick {sim.mission.cert_tick}  "
            f"clearance={sim.mission.cert_clearance:.2f} fine-cells  "
            f"path_len={len(sim.mission.corridor)} fine-cells"
        )
    else:
        print(f"\n⚠   Not certified after {sim.mission.tick} ticks.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",    type=int,   default=42)
    parser.add_argument("--no-anim", action="store_true")
    parser.add_argument("--hazards", type=int,   default=None)
    parser.add_argument("--delay",   type=float, default=None)
    args = parser.parse_args()

    cfg = Config()
    if args.hazards is not None:
        cfg.n_hazards  = args.hazards
    if args.delay is not None:
        cfg.tick_delay = args.delay

    print(
        f"Seed={args.seed}  "
        f"blocks={cfg.block_cols}x{cfg.block_rows}  "
        f"fine={cfg.fine_cols}x{cfg.fine_rows}  "
        f"K=4  hazards={cfg.n_hazards}"
    )

    sim, gt_map = _make_simulator(cfg, args.seed)

    if args.no_anim:
        run_headless(sim, cfg)
    else:
        run_animated(sim, gt_map, cfg)


if __name__ == "__main__":
    main()
