"""Pure flight-safety decisions used by the ROS node and unit tests."""

from typing import Optional


def landing_grounded(
    altitude_m: float,
    vertical_velocity_mps: float,
    ground_height_m: float,
    height_tolerance_m: float,
    velocity_tolerance_mps: float,
) -> bool:
    return (
        altitude_m <= ground_height_m + height_tolerance_m
        and abs(vertical_velocity_mps) <= velocity_tolerance_mps
    )


def landing_action(actuation: bool, grounded: bool, timed_out: bool) -> str:
    if not actuation:
        return "ALREADY_DISARMED"
    if grounded:
        return "DISARM"
    if timed_out:
        return "RETRY_LAND"
    return "WAIT"


def flight_time_expired(
    armed_at: Optional[float], now: float, max_flight_time_s: float
) -> bool:
    """Return true at the hard flight-time limit; zero disables dry-run only."""
    if armed_at is None or max_flight_time_s <= 0.0:
        return False
    return now - armed_at >= max_flight_time_s
