"""
Altitude-dependent sensing (ground footprint / range).

Used by exploration now; extended later for full camera FOV cones and RL obs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class DroneSensorModel:
    """
    Pinhole camera looking roughly nadir (mines on z=0).

    `ref_ground_range_m` matches the old flat `--sensor-range` at `ref_altitude_m`.
    """

    ref_altitude_m: float = 1.5
    ref_ground_range_m: float = 4.0
    camera_hfov_deg: float = 62.0
    camera_vfov_deg: float = 48.0
    camera_pitch_deg: float = -45.0  # below horizontal; -90 = straight down

    def ground_footprint_radius_m(self, altitude_m: float) -> float:
        """Approx. horizontal radius on the ground visible to the camera."""
        if altitude_m <= 0.05:
            return 0.2
        pitch = math.radians(self.camera_pitch_deg)
        # Nadir component of view axis
        down = max(0.15, -math.sin(pitch))
        hfov = math.radians(self.camera_hfov_deg)
        vfov = math.radians(self.camera_vfov_deg)
        half_span = max(math.tan(hfov / 2), math.tan(vfov / 2))
        footprint = altitude_m * down * half_span * 2.2
        scaled = self.ref_ground_range_m * (altitude_m / self.ref_altitude_m)
        return max(footprint, scaled * 0.5)

    def can_detect_ground_point(
        self,
        *,
        drone_x: float,
        drone_y: float,
        drone_z: float,
        point_x: float,
        point_y: float,
        point_z: float = 0.0,
    ) -> bool:
        horiz = math.hypot(point_x - drone_x, point_y - drone_y)
        radius = self.ground_footprint_radius_m(drone_z)
        if horiz > radius:
            return False
        slant = math.hypot(horiz, drone_z - point_z)
        max_slant = math.hypot(radius, drone_z)
        return slant <= max_slant + 0.5
