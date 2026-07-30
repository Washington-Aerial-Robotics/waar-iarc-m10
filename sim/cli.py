from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .config import MissionSimConfig
from .exploration import ExplorationSim
from .field import HumanPathField
from .mines import generate_random_mines, load_mines_from_csv, load_mines_from_json
from .pathfinding import plan_human_path, path_length_m
from .replay import mines_by_timestamp
from .visualize import run_explore_animation, save_human_path_plot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="IARC mission sim: human path from discovered mines",
    )
    parser.add_argument("--mines-csv", type=Path, help="SLAM mine_detections.csv (instant plan)")
    parser.add_argument("--mines-json", type=Path, help="JSON list of fused mines")
    parser.add_argument("--random-mines", type=int, default=0, help="Generate N random mines")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--min-confidence", type=float, default=0.1)
    parser.add_argument("--field-x", type=float, default=91.44, help="Field length / downrange (m), 300 ft")
    parser.add_argument("--field-y", type=float, default=24.38, help="Field width (m), 80 ft")
    parser.add_argument("--resolution", type=float, default=0.2)
    parser.add_argument(
        "--clearance",
        type=float,
        default=0.3048,
        help="Mine inflation radius (m); IARC default = 1 ft = 0.3048",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("sim/human_path.png"),
        help="Path plot output image",
    )
    parser.add_argument(
        "--export-path",
        type=Path,
        help="Write path waypoints as JSON [(x_m, y_m), ...]",
    )

    # Phase 2 — exploration
    parser.add_argument(
        "--explore",
        action="store_true",
        help="Drones discover mines over time, replan human path",
    )
    parser.add_argument(
        "--replay-csv",
        type=Path,
        help="Discover mines in timestamp order from perception log (use with --explore)",
    )
    parser.add_argument(
        "--ticks",
        type=int,
        default=None,
        help="Max simulation ticks (default: survey window = 7 min = 1680 ticks at 0.25 s)",
    )
    parser.add_argument("--drones", type=int, default=4, help="Patrol drones (IARC team size)")
    parser.add_argument("--sensor-range", type=float, default=4.0, help="Ground range at ref altitude (m)")
    parser.add_argument("--default-altitude", type=float, default=1.5, help="Cruise altitude (m AGL)")
    parser.add_argument("--min-altitude", type=float, default=0.4)
    parser.add_argument("--max-altitude", type=float, default=3.0)
    parser.add_argument(
        "--min-separation-soft",
        type=float,
        default=4.0,
        help="RL shaping: crowding penalty below this horizontal distance (m)",
    )
    parser.add_argument(
        "--min-separation-hard",
        type=float,
        default=1.5,
        help="Safety floor: pairs closer than this are hard violations (m)",
    )
    parser.add_argument("--animate", action="store_true", help="Live matplotlib viewer")
    parser.add_argument(
        "--delay",
        type=float,
        default=None,
        help=(
            "Seconds to pause after each rendered frame. "
            "Default uses physics dt (0.25 s) so wall-clock ≈ sim_time / time-scale."
        ),
    )
    parser.add_argument(
        "--time-scale",
        type=float,
        default=14.0,
        help=(
            "Playback only: shorten frame pause by N (default 14 → 7 min ≈ 30 s wall-clock). "
            "Draws every physics tick for smooth motion; does NOT change drone speed."
        ),
    )
    parser.add_argument(
        "--show-truth",
        action="store_true",
        default=True,
        help="Light-grey X for undiscovered mines (default on)",
    )
    parser.add_argument(
        "--hide-truth",
        action="store_true",
        help="Hide undiscovered mine markers",
    )
    parser.add_argument(
        "--no-animate",
        action="store_true",
        help="Run exploration headless (still saves --output at end)",
    )
    parser.add_argument(
        "--legacy-patrol",
        action="store_true",
        help="Old grid-step patrol instead of motor-mixer flight model",
    )
    parser.add_argument(
        "--serpentine-patrol",
        action="store_true",
        help="Fall back to legacy SerpentinePatrol (diagonal chase) instead of LaneCoveragePlanner",
    )
    parser.add_argument(
        "--search-speed",
        type=float,
        default=2.0,
        help="Lane coverage cruise speed along X (m/s); tunable placeholder for camera search",
    )
    parser.add_argument(
        "--num-passes",
        type=int,
        default=2,
        help="Lane coverage legs before landing (default 2 = outbound + offset return)",
    )
    return parser


def load_truth_mines(args: argparse.Namespace) -> list:
    if args.mines_csv:
        mines = load_mines_from_csv(args.mines_csv, min_confidence=args.min_confidence)
        print(f"Loaded {len(mines)} mines from {args.mines_csv}")
        return mines
    if args.mines_json:
        mines = load_mines_from_json(args.mines_json)
        print(f"Loaded {len(mines)} mines from {args.mines_json}")
        return mines
    if args.random_mines > 0:
        mines = generate_random_mines(
            args.random_mines,
            args.field_x,
            args.field_y,
            margin_m=1.0,
            seed=args.seed,
        )
        print(f"Generated {len(mines)} random mines (seed={args.seed})")
        return mines
    if args.replay_csv:
        events = mines_by_timestamp(args.replay_csv, args.min_confidence)
        mines = [m for _, m in events]
        print(f"Replay: {len(mines)} unique tags in {args.replay_csv}")
        return mines
    print(
        "No mine source — use --random-mines, --mines-json, --mines-csv, or --replay-csv",
        file=sys.stderr,
    )
    sys.exit(1)


def run_instant_plan(args: argparse.Namespace, config: MissionSimConfig, mines: list) -> int:
    field = HumanPathField(config)
    field.add_mines(mines)
    result = plan_human_path(field)
    if not result.found:
        print("NO SAFE PATH FOUND — discovered/known mines block all start→far corridors.")
        save_human_path_plot(field, None, args.output, title="NO SAFE PATH FOUND")
        return 1
    print(result.summary())
    save_human_path_plot(
        field,
        result.path_cells,
        args.output,
        title=result.summary(),
        path_width_m=result.width_m,
    )
    print(f"Saved plot to {args.output}")
    if args.export_path:
        _export_path(field, result.path_cells, args.export_path)
    return 0


def run_exploration(args: argparse.Namespace, config: MissionSimConfig, truth_mines: list) -> int:
    if args.replay_csv:
        return _run_replay(args, config, truth_mines)

    show_truth = bool(args.show_truth) and not bool(args.hide_truth)
    max_ticks = args.ticks if args.ticks is not None else config.survey_limit_ticks

    sim = ExplorationSim(
        config,
        truth_mines,
        num_drones=args.drones,
        sensor_range_m=args.sensor_range,
        legacy_patrol=args.legacy_patrol,
        serpentine_patrol=args.serpentine_patrol,
    )

    if args.animate and not args.no_animate:
        metrics = run_explore_animation(
            sim,
            max_ticks=max_ticks,
            delay_s=args.delay,
            show_truth=show_truth,
            output=args.output,
            time_scale=args.time_scale,
        )
        print(metrics.summary())
        if sim.path and args.export_path:
            _export_path(sim.field, sim.path, args.export_path)
        return 0 if metrics.path_found else 1

    for _ in range(max_ticks):
        metrics = sim.step()
        if metrics.survey_complete or metrics.survey_over:
            break
        if metrics.mines_discovered == metrics.mines_total and metrics.path_found:
            # Still finish coverage passes — do not early-exit on mines alone
            pass

    print(metrics.summary())
    path_title = metrics.summary()
    if metrics.mines_discovered > 0 and not metrics.path_found:
        path_title = "NO SAFE PATH FOUND — " + metrics.summary()
    save_human_path_plot(
        sim.field,
        sim.path,
        args.output,
        truth_mines=truth_mines if show_truth else None,
        drones=sim.drones,
        sensor=sim.sensor,
        title=path_title,
        path_width_m=float(getattr(sim.path_result, "width_m", 0.0) or 0.0),
    )
    print(f"Saved plot to {args.output}")
    if sim.path and args.export_path:
        _export_path(sim.field, sim.path, args.export_path)
    return 0 if metrics.path_found else 1


def _run_replay(args: argparse.Namespace, config: MissionSimConfig, truth_mines: list) -> int:
    events = mines_by_timestamp(args.replay_csv, args.min_confidence)
    if not events:
        print("No replay events in CSV", file=sys.stderr)
        return 1

    sim = ExplorationSim(config, truth_mines, num_drones=0, sensor_range_m=0)
    sim.drones = []

    if args.animate and not args.no_animate:
        import matplotlib.pyplot as plt

        cfg = config
        fig, ax = plt.subplots(figsize=(14, 3.5))
        plt.ion()
        from .visualize import _rgb_grid

        img = ax.imshow(
            _rgb_grid(sim.field),
            origin="lower",
            extent=[0, cfg.field_x_m, 0, cfg.field_y_m],
            aspect="equal",
        )
        path_line, = ax.plot([], [], color="#00e5ff", linewidth=2)
        disc = ax.plot([], [], "x", color="white", markersize=8)[0]
        title = ax.set_title("CSV replay")
        for i, (_ts, mine) in enumerate(events):
            sim.discover_from_csv_row(mine)
            sim.metrics.ticks = i + 1
            sim.metrics.mines_discovered = len(sim.discovered)
            sim.metrics.path_found = sim.path is not None
            sim.metrics.path_length_m = path_length_m(sim.field, sim.path) if sim.path else 0.0
            img.set_data(_rgb_grid(sim.field))
            if sim.path:
                pts = [sim.field.cell_to_world(r, c) for r, c in sim.path[::5]]
                if pts:
                    xs, ys = zip(*pts)
                    path_line.set_data(xs, ys)
                else:
                    path_line.set_data([], [])
            else:
                path_line.set_data([], [])
            disc.set_data(
                [m.world_x for m in sim.discovered.values()],
                [m.world_y for m in sim.discovered.values()],
            )
            title.set_text(sim.metrics.summary())
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(args.delay)
        plt.ioff()
        save_human_path_plot(sim.field, sim.path, args.output, title=sim.metrics.summary())
        plt.close(fig)
        print(sim.metrics.summary())
        return 0 if sim.path else 1

    for i, (_ts, mine) in enumerate(events):
        sim.discover_from_csv_row(mine)
        sim.metrics.ticks = i + 1
    sim.metrics.mines_discovered = len(sim.discovered)
    sim.metrics.path_found = sim.path is not None
    sim.metrics.path_length_m = path_length_m(sim.field, sim.path) if sim.path else 0.0
    print(sim.metrics.summary())
    save_human_path_plot(sim.field, sim.path, args.output)
    return 0 if sim.path else 1


def _export_path(field: HumanPathField, path: list, export_path: Path) -> None:
    waypoints = [
        {"x_m": field.cell_to_world(r, c)[0], "y_m": field.cell_to_world(r, c)[1]}
        for r, c in path
    ]
    export_path.parent.mkdir(parents=True, exist_ok=True)
    export_path.write_text(json.dumps(waypoints, indent=2), encoding="utf-8")
    print(f"Exported waypoints to {export_path}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    config = MissionSimConfig(
        field_x_m=args.field_x,
        field_y_m=args.field_y,
        resolution_m=args.resolution,
        clearance_m=args.clearance,
        default_altitude_m=args.default_altitude,
        min_altitude_m=args.min_altitude,
        max_altitude_m=args.max_altitude,
        min_separation_soft_m=args.min_separation_soft,
        min_separation_hard_m=args.min_separation_hard,
        search_speed_m_s=args.search_speed,
        num_passes=args.num_passes,
    )

    if args.explore or args.replay_csv:
        if args.replay_csv and args.random_mines == 0 and not args.mines_json:
            truth = load_truth_mines(args)
        else:
            truth = load_truth_mines(args)
        return run_exploration(args, config, truth)

    mines = load_truth_mines(args)
    return run_instant_plan(args, config, mines)


if __name__ == "__main__":
    raise SystemExit(main())
