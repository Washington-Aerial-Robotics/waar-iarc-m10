from drone_hardware_bridge.safety import (
    flight_time_expired, landing_action, landing_grounded,
)


def test_airborne_timeout_retries_land_and_never_disarms():
    assert landing_action(True, False, True) == "RETRY_LAND"


def test_disarm_requires_grounded_or_firmware_already_disarmed():
    assert landing_action(True, True, False) == "DISARM"
    assert landing_action(False, False, False) == "ALREADY_DISARMED"
    assert landing_action(True, False, False) == "WAIT"
    assert landing_grounded(0.1, 0.05, 0.0, 0.12, 0.15)
    assert not landing_grounded(2.0, 0.0, 0.0, 0.12, 0.15)


def test_maximum_flight_time_is_a_hard_boundary():
    assert not flight_time_expired(None, 500.0, 420.0)
    assert not flight_time_expired(10.0, 429.999, 420.0)
    assert flight_time_expired(10.0, 430.0, 420.0)
    assert flight_time_expired(10.0, 431.0, 420.0)
    # Zero is useful for an unlimited dry-run but is rejected by the hardware node.
    assert not flight_time_expired(10.0, 10_000.0, 0.0)
