from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from .exploration import ExplorationMetrics, ExplorationSim
from .field import HumanPathField
from .perception_geometry import DroneSensorModel
from .types import FREE, HAZARD, INFLATED

# Distinct markers when running four competition drones
DRONE_COLORS = ("#7fff00", "#00e5ff", "#ff6ec7", "#ffb347")


def _drone_color(index: int) -> str:
    return DRONE_COLORS[index % len(DRONE_COLORS)]


def _rgb_grid(field: HumanPathField) -> np.ndarray:
    display = np.zeros((field.config.rows, field.config.cols, 3), dtype=np.float32)
    display[field.grid == FREE] = (0.15, 0.18, 0.12)
    display[field.grid == INFLATED] = (0.55, 0.35, 0.1)
    display[field.grid == HAZARD] = (0.85, 0.15, 0.15)
    return display


def save_human_path_plot(
    field: HumanPathField,
    path: list[tuple[int, int]] | None,
    output: Path,
    *,
    truth_mines=None,
    drones=None,
    sensor: DroneSensorModel | None = None,
    title: str = "Human path over discovered mines",
) -> None:
    _render_frame(
        field,
        path,
        truth_mines=truth_mines,
        drones=drones,
        sensor=sensor,
        title=title,
        save_path=output,
    )


def run_explore_animation(
    sim: ExplorationSim,
    *,
    max_ticks: int,
    delay_s: float,
    show_truth: bool,
    output: Path | None,
) -> ExplorationMetrics:
    cfg = sim.config
    fig, (ax_map, ax_side) = plt.subplots(
        2,
        1,
        figsize=(14, 7),
        gridspec_kw={"height_ratios": [2.4, 1]},
    )
    plt.ion()
    img = ax_map.imshow(
        _rgb_grid(sim.field),
        origin="lower",
        extent=[0, cfg.field_x_m, 0, cfg.field_y_m],
        aspect="equal",
        interpolation="nearest",
    )
    path_line, = ax_map.plot([], [], color="#00e5ff", linewidth=2, label="human path")
    drone_dots = [
        ax_map.plot([], [], "o", color=_drone_color(i), markersize=6)[0]
        for i, _ in enumerate(sim.drones)
    ]
    drone_headings = [
        ax_map.plot([], [], "-", color=_drone_color(i), linewidth=1.5, alpha=0.85)[0]
        for i, _ in enumerate(sim.drones)
    ]
    footprint_patches = [
        Circle((0, 0), 1.0, fill=False, linewidth=1, linestyle="--", alpha=0.45)
        for _ in sim.drones
    ]
    for i, patch in enumerate(footprint_patches):
        patch.set_edgecolor(_drone_color(i))
        ax_map.add_patch(patch)
    truth_scatter = ax_map.plot([], [], "x", color="#666666", markersize=6, alpha=0.5)[0]
    discovered_scatter = ax_map.plot([], [], "x", color="white", markersize=8, markeredgewidth=2)[0]
    ax_map.axvline(cfg.edge_margin_m, color="lime", linestyle="--", alpha=0.5)
    ax_map.axvline(cfg.field_x_m - cfg.edge_margin_m, color="yellow", linestyle="--", alpha=0.5)
    ax_map.set_xlim(0, cfg.field_x_m)
    ax_map.set_ylim(0, cfg.field_y_m)
    ax_map.legend(loc="upper right", fontsize=7)

    ax_side.set_xlim(0, cfg.field_x_m)
    ax_side.set_ylim(0, cfg.max_altitude_m + 0.5)
    ax_side.set_ylabel("altitude (m)")
    ax_side.set_xlabel("x downrange (m)")
    ax_side.axhline(cfg.ground_z_m, color="#444", linewidth=1)
    alt_dots = [
        ax_side.plot([], [], "o", color=_drone_color(i), markersize=5)[0]
        for i, _ in enumerate(sim.drones)
    ]
    alt_trails = [
        ax_side.plot([], [], "-", color=_drone_color(i), linewidth=0.8, alpha=0.5)[0]
        for i, _ in enumerate(sim.drones)
    ]

    status = fig.suptitle("Exploration sim", fontsize=10)
    plt.tight_layout()

    last_metrics = ExplorationMetrics(mines_total=sim.metrics.mines_total)
    for _ in range(max_ticks):
        last_metrics = sim.step()
        img.set_data(_rgb_grid(sim.field))
        if sim.path:
            xs, ys = [], []
            for row, col in sim.path[:: max(1, len(sim.path) // 200)]:
                x, y = sim.field.cell_to_world(row, col)
                xs.append(x)
                ys.append(y)
            path_line.set_data(xs, ys)
        else:
            path_line.set_data([], [])
        for i, (dot, heading, patch, drone) in enumerate(
            zip(drone_dots, drone_headings, footprint_patches, sim.drones)
        ):
            color = _drone_color(i)
            dot.set_data([drone.x], [drone.y])
            hx = drone.x + 0.9 * math.cos(drone.yaw)
            hy = drone.y + 0.9 * math.sin(drone.yaw)
            heading.set_data([drone.x, hx], [drone.y, hy])
            radius = sim.sensor.ground_footprint_radius_m(drone.z)
            patch.center = (drone.x, drone.y)
            patch.set_radius(radius)
            patch.set_edgecolor(color)
            alt_dots[i].set_data([drone.x], [drone.z])
            if drone.trail:
                alt_trails[i].set_data(
                    [p[0] for p in drone.trail[-120:]],
                    [p[2] for p in drone.trail[-120:]],
                )
        if sim.discovered:
            discovered_scatter.set_data(
                [m.world_x for m in sim.discovered.values()],
                [m.world_y for m in sim.discovered.values()],
            )
        if show_truth:
            hidden = [m for m in sim.truth_mines if m.tag_id not in sim.discovered]
            if hidden:
                truth_scatter.set_data([m.world_x for m in hidden], [m.world_y for m in hidden])
            else:
                truth_scatter.set_data([], [])
        status.set_text(last_metrics.summary())
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(delay_s)

    plt.ioff()
    if output is not None:
        save_human_path_plot(
            sim.field,
            sim.path,
            output,
            truth_mines=sim.truth_mines if show_truth else None,
            drones=sim.drones,
            sensor=sim.sensor,
            title=f"Final — {last_metrics.summary()}",
        )
    else:
        plt.show(block=True)
    plt.close(fig)
    return last_metrics


def _render_frame(
    field: HumanPathField,
    path: list[tuple[int, int]] | None,
    *,
    truth_mines,
    drones,
    sensor: DroneSensorModel | None,
    title: str,
    save_path: Path | None,
) -> None:
    cfg = field.config
    fig, (ax_map, ax_side) = plt.subplots(
        2,
        1,
        figsize=(14, 6.5),
        gridspec_kw={"height_ratios": [2.4, 1]},
    )
    ax_map.imshow(
        _rgb_grid(field),
        origin="lower",
        extent=[0, cfg.field_x_m, 0, cfg.field_y_m],
        aspect="equal",
        interpolation="nearest",
    )
    if truth_mines:
        for mine in truth_mines:
            if mine.tag_id not in {m.tag_id for m in field.mines}:
                ax_map.plot(mine.world_x, mine.world_y, "x", color="#666666", markersize=6, alpha=0.5)
    for mine in field.mines:
        ax_map.plot(mine.world_x, mine.world_y, "x", color="white", markersize=8, markeredgewidth=2)
    if path:
        xs, ys = [], []
        for row, col in path:
            x, y = field.cell_to_world(row, col)
            xs.append(x)
            ys.append(y)
        ax_map.plot(xs, ys, color="#00e5ff", linewidth=2, label="human path")
    if drones:
        for i, drone in enumerate(drones):
            color = _drone_color(i)
            ax_map.plot(drone.x, drone.y, "o", color=color, markersize=6)
            hx = drone.x + 0.9 * math.cos(drone.yaw)
            hy = drone.y + 0.9 * math.sin(drone.yaw)
            ax_map.plot([drone.x, hx], [drone.y, hy], "-", color=color, linewidth=1.5, alpha=0.85)
            if sensor is not None:
                radius = sensor.ground_footprint_radius_m(drone.z)
                ax_map.add_patch(
                    Circle(
                        (drone.x, drone.y),
                        radius,
                        fill=False,
                        linestyle="--",
                        edgecolor=color,
                        alpha=0.45,
                        linewidth=1,
                    )
                )
            if drone.trail:
                ax_side.plot(
                    [p[0] for p in drone.trail],
                    [p[2] for p in drone.trail],
                    "-",
                    color=color,
                    linewidth=0.8,
                    alpha=0.6,
                )
            ax_side.plot(drone.x, drone.z, "o", color=color, markersize=5)
    ax_map.axvline(cfg.edge_margin_m, color="lime", linestyle="--", alpha=0.6)
    ax_map.axvline(cfg.field_x_m - cfg.edge_margin_m, color="yellow", linestyle="--", alpha=0.6)
    ax_map.set_xlabel("x (m)")
    ax_map.set_ylabel("y (m)")
    ax_side.set_xlim(0, cfg.field_x_m)
    ax_side.set_ylim(0, cfg.max_altitude_m + 0.5)
    ax_side.set_ylabel("altitude (m)")
    ax_side.set_xlabel("x downrange (m)")
    ax_side.axhline(cfg.ground_z_m, color="#444", linewidth=1)
    fig.suptitle(title, fontsize=9)
    ax_map.legend(loc="upper right", fontsize=8)
    plt.tight_layout()
    if save_path:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(save_path, dpi=200)
    plt.close(fig)
