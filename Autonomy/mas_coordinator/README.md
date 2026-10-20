# MAS Coordinator - IARC Mission 10

The coordinator runs one mission, task-auction, and belief-sync process per
drone. The current launch targets four drones in a 91.44 m x 24.38 m arena and
a 420 second mission.

## Runtime ownership

- `mas_mission` is the only owner of mission state. It publishes the current
  state on `/{drone_id}/mission_state` once per second.
- `mas_sync` is the only publisher of that drone's `PoseBeacon`. It does not
  publish until local pose is valid and stops when pose becomes stale. Mission's
  read-only belief mirror uses the same canonical `BeliefStore` merge rules.
- `mas_task` owns auctions and task execution state. It bids only after both a
  fresh local pose and an executable mission state are available.
- The autonomy/planner process owns vehicle motion, mine detection, and task
  completion. It must implement the planner-facing topics below.

This ownership rule is important: the three ROS nodes are separate processes,
so calling methods on another node cannot be used as an integration path.

## Packages

| Package | Responsibility |
| --- | --- |
| `mas_interfaces` | ROS 2 messages for beacons, beliefs, sync, and auctions |
| `mas_sync` | Valid-pose beacons, neighbor windows, belief anti-entropy/fusion |
| `mas_task` | Distributed auction, deterministic winner, retry, timeout |
| `mas_mission` | State machine, safety behavior tree, roles, final grid |

## Planner-facing contract

For each drone, the autonomy adapter must provide:

| Topic | Type | Direction | Requirement |
| --- | --- | --- | --- |
| `/{drone_id}/pose` | `geometry_msgs/PoseStamped` | planner -> MAS | Map-frame pose at a steady rate |
| `/{drone_id}/mine_candidates` | `mas_interfaces/MineBelief` | planner -> MAS | Unique `mine_id`, valid confidence, monotonic per-mine `seq` |
| `/{drone_id}/mission_cmd` | `std_msgs/String` JSON | MAS -> planner | Execute the command or retain the safe hold state |
| `/team/task_result` | `mas_interfaces/TaskResult` | planner -> MAS | Required after every assigned verification/path task |

`/{drone_id}/task_cmd` is an internal handoff from `mas_task` to
`mas_mission`. The mission behavior tree forwards it to `mission_cmd`.

Recognized command payloads emitted by MAS are:

- `HOLD_POSITION` with a `reason` for collision, geofence, or stale pose
- `SWEEP_SECTOR`, `STANDBY_FOR_TASK`, and `FILL_GAPS`
- a task payload containing `task_id`, `task_type`, target, and priority
- `AWAIT_PATH_VERIFY` and `VERIFY_PATH` (including the role task ID)
- `LAND_AND_SUBMIT`

For `VERIFY_TAG`, the planner must publish a `TaskResult` with the same
`task_id`, its `executor_id`, the original `mine_id`, and an outcome of
`confirmed`, `rejected`, `uncertain`, or `failed`. Uncertain/failed tasks and
timed-out tasks are retried by the original announcer with a new auction ID.

## Team topics

| Topic | Main publisher | Main consumers |
| --- | --- | --- |
| `/team/pose_beacon` | `mas_sync` | sync and mission nodes |
| `/team/sync_hello`, `/team/sync_ack` | `mas_sync` | sync nodes |
| `/team/mine_delta` | `mas_sync` | sync and mission nodes |
| `/team/task_announce` | task/mission nodes | task and mission nodes |
| `/team/task_claim` | task nodes | task nodes |
| `/team/task_result` | task winner or planner | task, sync, and mission nodes |

Belief synchronization sends the complete small mine snapshot during
anti-entropy. `known_mine_count` is retained for wire compatibility and
diagnostics; it is not used as a version cursor because equal counts do not
imply equal IDs or revisions.

## Auction and role behavior

- Repeated DDS announcements do not produce repeated bids.
- Only the original announcer retries an abandoned/failed task.
- Every retry uses a new task ID, so a closed auction is never reused.
- A running task is failed and released after `task_timeout_s`.
- Only `role_coordinator_id` (default `d1`) announces the two team-wide role
  auctions. The elected task node immediately publishes an `assigned` result,
  giving all mission nodes the same winner.

## Safety and final output

BOOT cannot exit until this drone has a fresh local pose and all expected peer
beacons are fresh. Safety guards use the adapter-supported `HOLD_POSITION`
command and never treat a zero-initialized pose as valid.

The final `safe_path_grid` now starts as unknown (`-1`) and marks confirmed
mines occupied (`100`). It no longer labels the entire unobserved arena safe.
Its origin quaternion is valid. A useful traversability grid still requires a
real SLAM/planner occupancy source; MAS alone cannot infer free space.

## Build and launch

```bash
cd /ros2_ws
colcon build --packages-select mas_interfaces
source install/setup.bash
colcon build --packages-select mas_sync mas_task mas_mission
source install/setup.bash

ros2 launch mas_mission team_launch.py
```

Useful overrides:

```bash
ros2 launch mas_mission team_launch.py \
  mission_duration:=420.0 \
  arena_width:=91.44 \
  arena_height:=24.38 \
  role_coordinator_id:=d1
```

Run the pure-Python tests from this directory:

```bash
python3 -m pytest tests -v
```

## Remaining external integration work

- The real autonomy adapter must execute every command listed above and publish
  `TaskResult`; the coordinator cannot complete physical tasks on its behalf.
- The final grid should ingest the real SLAM/planner occupancy layer before it
  is used as a scored safe-path map.
- A ROS 2 launch test with four adapters, message loss/delay, drone dropout, and
  task timeout should be run in a ROS-enabled environment before flight.
