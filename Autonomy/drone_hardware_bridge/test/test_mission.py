import json

from drone_hardware_bridge.grid_planner import Grid, GridPlanner
from drone_hardware_bridge.mission import CommandPlanner


def planner(aligned=True):
    command = CommandPlanner(10.0, 10.0, 2.0, aligned, clock=lambda: 5.0)
    occupancy = Grid(10, 10, 1.0, 0.0, 0.0, 0.0, tuple([0] * 100))
    return command, GridPlanner(occupancy, inflation_radius_m=0.0)


def test_unknown_and_uncalibrated_commands_hold():
    command, occupancy = planner()
    assert command.command('{"cmd":"BOGUS"}', (1.5, 1.5), occupancy).mode == "HOLD"
    command, occupancy = planner(aligned=False)
    assert command.command(
        '{"cmd":"SWEEP_SECTOR","x_min":1,"x_max":8,"y_min":1,"y_max":8}',
        (1.5, 1.5), occupancy,
    ).mode == "HOLD"


def test_sweep_and_fill_are_routed_through_grid():
    command, occupancy = planner()
    for name in ("SWEEP_SECTOR", "FILL_GAPS"):
        plan = command.command(json.dumps({
            "cmd": name, "x_min": 1.5, "x_max": 8.5,
            "y_min": 1.5, "y_max": 8.5,
        }), (1.5, 1.5), occupancy)
        assert plan.mode == "COVERAGE"
        assert plan.path


def test_verify_tag_requires_matching_explicit_result_and_never_auto_confirms():
    command, occupancy = planner()
    plan = command.command(json.dumps({
        "task_id": "t1", "task_type": "VERIFY_TAG", "mine_id": "m1",
        "target_x": 5.5, "target_y": 5.5,
    }), (1.5, 1.5), occupancy)
    assert plan.mode == "VERIFY_TAG"
    while plan.path:
        command.reached(plan.path[0], 0.01)
    assert command.verification_result(json.dumps({
        "mine_id": "other", "outcome": "confirmed", "confidence": 1.0,
        "x": 9.0, "y": 9.0,
    }), 0.5) is None
    assert command.verification_result(json.dumps({
        "mine_id": "m1", "outcome": "confirmed", "confidence": 0.9,
    }), 0.5) == ("t1", "m1", "confirmed", 0.9)


def test_land_is_explicit_plan_not_arm():
    command, occupancy = planner()
    assert command.command('{"cmd":"LAND_AND_SUBMIT"}', (1.5, 1.5), occupancy).mode == "LAND"


def test_task_cmd_payload_shape_routes_verify_tag():
    command, occupancy = planner()
    plan = command.command(json.dumps({
        "task_id": "won_1", "task_type": "VERIFY_TAG", "mine_id": "m2",
        "target_x": 3.5, "target_y": 4.5, "priority": 0.8,
    }), (1.5, 1.5), occupancy)
    assert plan.mode == "VERIFY_TAG"
    assert plan.task.task_id == "won_1"
