"""Occupancy-grid A* and deterministic coverage paths, independent of ROS."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, List, Sequence, Tuple


Cell = Tuple[int, int]
Point = Tuple[float, float]


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Tuple[int, ...]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError("grid dimensions and resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy data length does not match grid dimensions")

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, cell: Cell) -> int:
        if not self.in_bounds(cell):
            raise IndexError(cell)
        return self.data[cell[1] * self.width + cell[0]]

    def world_to_cell(self, point: Point) -> Cell:
        dx = point[0] - self.origin_x
        dy = point[1] - self.origin_y
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        return math.floor(local_x / self.resolution), math.floor(local_y / self.resolution)

    def cell_to_world(self, cell: Cell) -> Point:
        local_x = (cell[0] + 0.5) * self.resolution
        local_y = (cell[1] + 0.5) * self.resolution
        c = math.cos(self.origin_yaw)
        s = math.sin(self.origin_yaw)
        return (
            self.origin_x + c * local_x - s * local_y,
            self.origin_y + s * local_x + c * local_y,
        )


class GridPlanner:
    def __init__(
        self,
        grid: Grid,
        occupied_threshold: int = 65,
        unknown_is_blocked: bool = True,
        inflation_radius_m: float = 0.5,
    ) -> None:
        self.grid = grid
        self.occupied_threshold = occupied_threshold
        self.unknown_is_blocked = unknown_is_blocked
        self.inflation_radius_m = max(0.0, inflation_radius_m)
        self._blocked = self._inflate()

    def _raw_blocked(self, cell: Cell) -> bool:
        value = self.grid.value(cell)
        return value < 0 if self.unknown_is_blocked and value < 0 else value >= self.occupied_threshold

    def _inflate(self) -> set[Cell]:
        blocked = {
            (x, y)
            for y in range(self.grid.height)
            for x in range(self.grid.width)
            if self._raw_blocked((x, y))
        }
        radius = math.ceil(self.inflation_radius_m / self.grid.resolution)
        if radius == 0:
            return blocked
        inflated = set(blocked)
        radius_m_sq = self.inflation_radius_m * self.inflation_radius_m + 1e-12
        for x, y in blocked:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if (dx * self.grid.resolution) ** 2 + (dy * self.grid.resolution) ** 2 > radius_m_sq:
                        continue
                    candidate = (x + dx, y + dy)
                    if self.grid.in_bounds(candidate):
                        inflated.add(candidate)
        return inflated

    def is_free(self, cell: Cell) -> bool:
        return self.grid.in_bounds(cell) and cell not in self._blocked

    def plan(self, start_world: Point, goal_world: Point) -> List[Point]:
        start = self.grid.world_to_cell(start_world)
        goal = self.grid.world_to_cell(goal_world)
        if not self.is_free(start) or not self.is_free(goal):
            return []
        if start == goal:
            return [goal_world]

        frontier: list[tuple[float, int, Cell]] = []
        counter = 0
        heapq.heappush(frontier, (0.0, counter, start))
        came_from: dict[Cell, Cell | None] = {start: None}
        cost_so_far = {start: 0.0}
        moves = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for dx, dy, move_cost in moves:
                neighbor = current[0] + dx, current[1] + dy
                if not self.is_free(neighbor):
                    continue
                # Never cut a diagonal corner between occupied cells.
                if dx and dy and (
                    not self.is_free((current[0] + dx, current[1]))
                    or not self.is_free((current[0], current[1] + dy))
                ):
                    continue
                new_cost = cost_so_far[current] + move_cost
                if neighbor in cost_so_far and new_cost >= cost_so_far[neighbor]:
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                counter += 1
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(frontier, (new_cost + heuristic, counter, neighbor))

        if goal not in came_from:
            return []
        cells = []
        current: Cell | None = goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        # Start is already occupied by the vehicle. Preserve every subsequent
        # cell so the controller cannot shortcut through an unchecked region.
        points = [self.grid.cell_to_world(cell) for cell in cells[1:]]
        points.append(goal_world)
        return points

    def path_is_free(self, points: Sequence[Point]) -> bool:
        return bool(points) and all(self.is_free(self.grid.world_to_cell(point)) for point in points)


def lawnmower_waypoints(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    spacing_m: float,
) -> List[Point]:
    values = (x_min, x_max, y_min, y_max, spacing_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("coverage bounds must be finite")
    if x_max <= x_min or y_max <= y_min or spacing_m <= 0:
        raise ValueError("coverage bounds/spacing are invalid")
    rows = max(1, math.ceil((y_max - y_min) / spacing_m))
    ys = [min(y_max, y_min + index * spacing_m) for index in range(rows + 1)]
    if ys[-1] < y_max:
        ys.append(y_max)
    result: List[Point] = []
    for index, y in enumerate(ys):
        result.extend(((x_min, y), (x_max, y)) if index % 2 == 0 else ((x_max, y), (x_min, y)))
    return result
