import numpy as np
from scipy.spatial.transform import Rotation as R


class SparseVoxelMap:

    UNKNOWN  = 0
    FREE     = 1
    HAZARD   = 2
    INFLATED = 3

    def __init__(self, origin_row, resolution=0.2, clearance=0.3, field_x=94, field_y=12):

        self.resolution = resolution
        self.clearance = clearance

        self.temporary_voxels = set()
        self.permanent_cells = {}  # (row, col) -> UNKNOWN|FREE|HAZARD|INFLATED

        self.max_row = int(field_y / self.resolution)
        self.max_col = int(field_x / self.resolution)

        self.origin = np.array([origin_row["x"],
                                origin_row["y"],
                                origin_row["z"]])

        self.offsets_3d = self._compute_offsets_3d()
        self.offsets_2d = self._compute_offsets_2d()

    def get_neighbors_3d(self, row, col, layer):
        neighbors = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                for dl in (-1, 0, 1):
                    if dr == 0 and dc == 0 and dl == 0:
                        continue
                    nr, nc, nl = (row + dr, col + dc, layer + dl)
                    if not self._in_bounds_3d(nr, nc, nl):
                        continue
                    if (nr, nc, nl) not in self.temporary_voxels:
                        neighbors.append((nr, nc, nl))
        return neighbors

    def get_neighbors_2d(self, row, col):
        neighbors = []
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = row + dr, col + dc
                if not self._in_bounds_2d(nr, nc):
                    continue
                state = self.permanent_cells.get((nr, nc), self.UNKNOWN)
                if state in (self.HAZARD, self.INFLATED):
                    continue
                base_cost = 1.414 if (dr != 0 and dc != 0) else 1.0
                cost = base_cost if state == self.FREE else base_cost * 4.0
                neighbors.append(((nr, nc), cost))
        return neighbors

    def new_obstacle(self, obstacle_points, quaternion, gps_position):
        for obstacle_point in obstacle_points:
            rotated = self._rotate_point(obstacle_point, quaternion)
            world_point = gps_position + rotated
            row, col, layer = self._world_to_grid_3d(
                world_point[0], world_point[1], world_point[2]
            )
            self._inflate_3d(row, col, layer)

    def new_mine(self, mine_points, quaternion, gps_position):
        for mine_point in mine_points:
            point_3d = np.array([mine_point[0], mine_point[1], 0.0])
            rotated = self._rotate_point(point_3d, quaternion)
            world_x = gps_position[0] + rotated[0]
            world_y = gps_position[1] + rotated[1]
            row, col = self._world_to_grid_2d(world_x, world_y)
            self._inflate_2d(row, col)

    def add_mine_world(self, world_x: float, world_y: float):
        """Mark a discovered mine at world coordinates (meters)."""
        row, col = self._world_to_grid_2d(world_x, world_y)
        self._inflate_2d(row, col)

    def mark_free(self, x: float, y: float):
        row, col = self._world_to_grid_2d(x, y)
        if not self._in_bounds_2d(row, col):
            return
        if self.permanent_cells.get((row, col), self.UNKNOWN) not in (self.HAZARD, self.INFLATED):
            self.permanent_cells[(row, col)] = self.FREE

    def is_obstacle(self, x: float, y: float, z: float):
        row, col, layer = self._world_to_grid_3d(x, y, z)
        return (row, col, layer) in self.temporary_voxels

    def is_mine(self, x: float, y: float):
        row, col = self._world_to_grid_2d(x, y)
        return self.permanent_cells.get((row, col), self.UNKNOWN) in (self.HAZARD, self.INFLATED)

    def update_drone_position(self, gps_position):
        drone_row, drone_col, drone_layer = self._world_to_grid_3d(
            gps_position[0], gps_position[1], gps_position[2]
        )
        radius_cells = 4.5 / self.resolution
        self.temporary_voxels = {
            (row, col, layer)
            for (row, col, layer) in self.temporary_voxels
            if (row - drone_row)**2 + (col - drone_col)**2 + (layer - drone_layer)**2
            <= radius_cells**2
        }

    def generate_grid_3d(self):
        if not self.temporary_voxels:
            return np.zeros((self.max_row, self.max_col, 1), dtype=np.uint8)
        max_layer = max(layer for row, col, layer in self.temporary_voxels) + 1
        grid = np.zeros((self.max_row, self.max_col, max_layer), dtype=np.uint8)
        for (row, col, layer) in self.temporary_voxels:
            if self._in_bounds_3d(row, col, layer):
                grid[row, col, layer] = 1
        return grid

    def generate_grid_2d(self):
        grid = np.zeros((self.max_row, self.max_col), dtype=np.uint8)
        for (row, col), state in self.permanent_cells.items():
            if self._in_bounds_2d(row, col):
                grid[row, col] = state
        return grid

    def generate_visualization(self):
        import matplotlib.pyplot as plt

        grid_3d = self.generate_grid_3d()
        proj    = np.max(grid_3d, axis=2)
        grid_2d = self.generate_grid_2d()

        max_x = self.max_col * self.resolution
        max_y = self.max_row * self.resolution

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

        ax1.imshow(proj, origin="lower", interpolation="nearest",
                   extent=[0, max_x, 0, max_y])
        ax1.set_title("3D Obstacle Map (drone navigation)")
        ax1.set_xlabel("x (meters)")
        ax1.set_ylabel("y (meters)")

        ax2.imshow(grid_2d, origin="lower", interpolation="nearest",
                   extent=[0, max_x, 0, max_y])
        ax2.set_title("2D Mine Map (human path planning)")
        ax2.set_xlabel("x (meters)")
        ax2.set_ylabel("y (meters)")

        plt.tight_layout()
        plt.savefig("occ_grid_proj.png", dpi=200)
        plt.close()

        np.save("occ_grid_3d.npy", grid_3d)
        np.save("occ_grid_2d.npy", grid_2d)

    def _in_bounds_3d(self, row, col, layer):
        return (0 <= row < self.max_row and 0 <= col < self.max_col and layer >= 0)

    def _in_bounds_2d(self, row, col):
        return (0 <= row < self.max_row and 0 <= col < self.max_col)

    def _rotate_point(self, point, quaternion):
        r = R.from_quat(quaternion)
        rotated = r.apply(point)
        rotated = rotated.copy()
        #rotated[2] = -rotated[2]
        return rotated

    def _world_to_grid_3d(self, x, y, z):
        row   = int(np.floor(y / self.resolution))
        col   = int(np.floor(x / self.resolution))
        layer = int(np.floor(z / self.resolution))
        return row, col, layer

    def _world_to_grid_2d(self, x, y):
        row = int(np.floor(y / self.resolution))
        col = int(np.floor(x / self.resolution))
        return row, col

    def _grid_to_world_3d(self, row, col, layer):
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        z = (layer + 0.5) * self.resolution
        return x, y, z

    def _grid_to_world_2d(self, row, col):
        x = (col + 0.5) * self.resolution
        y = (row + 0.5) * self.resolution
        return x, y

    def _inflate_3d(self, row, col, layer):
        for dr, dc, dl in self.offsets_3d:
            nr, nc, nl = row + dr, col + dc, layer + dl
            if self._in_bounds_3d(nr, nc, nl):
                self.temporary_voxels.add((nr, nc, nl))

    def _inflate_2d(self, row, col):
        if self._in_bounds_2d(row, col):
            self.permanent_cells[(row, col)] = self.HAZARD
        for dr, dc in self.offsets_2d:
            if dr == 0 and dc == 0:
                continue
            nr, nc = row + dr, col + dc
            if self._in_bounds_2d(nr, nc):
                if self.permanent_cells.get((nr, nc), self.UNKNOWN) != self.HAZARD:
                    self.permanent_cells[(nr, nc)] = self.INFLATED

    def _compute_offsets_3d(self):
        r_cells = int(np.ceil(self.clearance / self.resolution))
        offsets = []
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                for dl in range(-r_cells, r_cells + 1):
                    if (dr**2 + dc**2 + dl**2) <= r_cells**2:
                        offsets.append((dr, dc, dl))
        return offsets

    def _compute_offsets_2d(self):
        r_cells = int(np.ceil(self.clearance / self.resolution))
        offsets = []
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if (dr**2 + dc**2) <= r_cells**2:
                    offsets.append((dr, dc))
        return offsets