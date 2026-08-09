# MAS Coordinator development notes

Read `README.md` first. It contains the current ROS topic and planner contract.
This file records implementation invariants that future changes must preserve.

## Process boundaries

Each drone runs three independent ROS 2 processes:

- `mission_logic_node` owns the mission state and publishes
  `/{drone_id}/mission_state`.
- `p2p_sync_node` owns the drone's `PoseBeacon` and replicated belief store.
- `p2p_task_node` owns auctions, retries, and task execution state.

Do not use public Python methods as a cross-node integration mechanism. State,
mine candidates, and task results must travel over ROS topics.

## Safety invariants

- BOOT readiness requires a fresh local pose and the configured number of fresh
  peer beacons.
- Sync never publishes a zero-initialized or stale pose.
- Task bidding fails closed without a fresh pose or recognized mission state.
- Safety guards publish `HOLD_POSITION`; an unknown command must never release
  a hold in the autonomy adapter.
- The final occupancy grid starts unknown, never implicitly free, and has a
  valid origin quaternion.

## Belief invariants

- Local `mine_candidates` must be inserted into `BeliefStore` and published as
  `MineDelta`.
- Only the result executor's sync node versions a `TaskResult` into a belief.
- Confirmed beliefs are never downgraded by rejected/candidate updates.
- Equal versions use status precedence (`confirmed > rejected > uncertain >
  candidate`) and then confidence.
- `known_mine_count` is not a version cursor. Anti-entropy sends the complete
  small mine snapshot to guarantee convergence.
- Relay only beliefs that changed locally, to prevent duplicate relay storms.

## Auction invariants

- A duplicate announcement returns `False` from `AuctionManager.on_announce`
  and cannot produce a duplicate bid.
- Only the original announcer retries abandoned, failed, uncertain, or timed-out
  work.
- Each retry has a new task ID; a closed auction ID is never reopened.
- Role auctions are announced once by `role_coordinator_id` (default `d1`).
- The winning task node immediately publishes an `assigned` result for role
  tasks, so every mission node learns the same winner.
- Normal tasks must eventually produce `TaskResult`; executor timeout releases
  `busy` and triggers announcer-owned retry.

## Planner integration

The external autonomy adapter must publish `/{drone_id}/pose`, publish local
`mine_candidates`, consume `/{drone_id}/mission_cmd`, and publish
`/team/task_result`. The coordinator does not directly command a flight
controller. Do not add a second `PoseBeacon` publisher to the adapter.

The current coordinator emits these JSON command families:

- `HOLD_POSITION`, `LAND_AND_SUBMIT`
- `SWEEP_SECTOR`, `STANDBY_FOR_TASK`, `FILL_GAPS`
- `AWAIT_PATH_VERIFY`, `VERIFY_PATH`
- task payloads with `task_id`, `task_type`, target, and priority

Any adapter change must handle every command explicitly and remain held on an
unknown command.

## Validation

From `Autonomy/mas_coordinator`:

```bash
python3 -m pytest tests -q
```

In a sourced ROS 2 workspace:

```bash
colcon build --packages-select mas_interfaces mas_sync mas_task mas_mission
ros2 launch mas_mission team_launch.py --show-args
```

Before flight, add/run a four-drone ROS launch test with command completion,
message delay/loss, localization dropout, and task timeout.
