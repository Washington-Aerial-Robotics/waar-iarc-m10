# MAS Coordinator — CLAUDE.md

## Project Overview

Multi-agent drone coordination system for IARC Mission 10 (300 ft × 80 ft arena, 7-minute mission, 4 drones). Each drone runs three independent ROS2 nodes: `p2p_sync_node` (belief synchronisation via a Scheme-B sync-window protocol), `p2p_task_node` (distributed task auction), and `mission_logic_node` (state machine + behaviour tree). A fourth component, `ros2_adapter_v2.py`, lives in Kevin's `waar_autonomy` repo and bridges his `BaselineLoop` path planner to the MAS topics. All four MAS nodes are pure-Python, use only `mas_interfaces` custom messages, and are designed to run without a centralised coordinator.

---

## Package Structure

```
mas_coordinator/
  mas_interfaces/                     Custom ROS2 message definitions (ament_cmake)
    msg/MineBelief.msg                mine_id, x, y, confidence, status, last_updated_by, seq, stamp
    msg/PoseBeacon.msg                drone_id, x, y, z, heading_deg, state, battery_pct, stamp
    msg/MineDelta.msg                 sender_id, beliefs[], ttl, stamp  — batch sync payload
    msg/SyncHello.msg                 sender_id, target_id, known_mine_count, stamp
    msg/SyncAck.msg                   sender_id, target_id, known_mine_count, stamp
    msg/TaskAnnounce.msg              task_id, task_type, announcer_id, target_x/y, priority, claim_window_s, stamp
    msg/TaskClaim.msg                 task_id, bidder_id, cost, stamp
    msg/TaskResult.msg                task_id, executor_id, outcome, mine_id, confidence, stamp

  mas_sync/
    mas_sync/p2p_sync_node.py         ROS2 node: beacon broadcast, neighbor graph, sync-window protocol, MineDelta relay
    mas_sync/belief_fusion.py         Pure Python: BeliefStore + merge rules (seq-LWW, confirmed sticky, confirmed>rejected)

  mas_task/
    mas_task/p2p_task_node.py         ROS2 node: mine candidate → auction announce, bid, resolve, retry, task dispatch
    mas_task/auction_manager.py       Pure Python: AuctionEntry, AuctionManager (open/close/won/abandoned), compute_cost()

  mas_mission/
    mas_mission/state_machine.py      Pure Python: StateMachine BOOT→SURVEY→VERIFY_TAG→PATH_VERIFY→CONVERGE→FINISH
    mas_mission/bt_runner.py          Pure Python: PrioritySelector BT, 6 nodes (Collision, Geofence, Failure, TaskExec, Exploration, P2PSync)
    mas_mission/mission_logic_node.py ROS2 node: owns SM + BT, issues mission cmds, tracks team/mines, publishes occupancy grid
    mas_mission/stub_explorer.py      ROS2 node: lawnmower pose at 5 Hz, single mine candidate after 30 s — integration test stub
    launch/team_launch.py             Launches 3 nodes × 4 drones (sync + task + mission); arena/duration as overridable args

  tests/
    test_auction_manager.py           Unit tests for AuctionManager and compute_cost()
    test_belief_fusion.py             Unit tests for BeliefStore merge rules
    test_bt_runner.py                 Unit tests for all 6 BT nodes via Mock (no ROS2)
    test_state_machine.py             Unit tests for all StateMachine transitions
    test_edge_cases.py                Edge-case tests: arena dims, 420 s, no-bidder retry, dropout, belief conflict

waar_autonomy/src/adapters/
    ros2_adapter_v2.py                Bridges Kevin's BaselineLoop to ROS2 topics (all 4 drones in one node)
```

---

## Implementation Status

### mas_interfaces
- **Implemented:** All 8 message types defined and building.
- **Missing:** No service or action types. `TaskResult` has no `task_type` field — callers must maintain a local `_task_registry: Dict[str, str]` mapping `task_id → task_type`.

### mas_sync: p2p_sync_node
- **Implemented:**
  - Beacon broadcast at configurable rate (default 5 Hz) to `/team/pose_beacon`
  - Neighbor graph with hysteresis (`r_enter=8m`, `r_exit=12m`) and 3 s stale-prune
  - Scheme-B sync window: SyncHello → SyncAck → MineDelta exchange
  - TTL-based MineDelta relay (TTL=2, decremented on relay)
  - Own pose update via `/{drone_id}/pose` (PoseStamped from explorer)
- **Missing / Stub:**
  - `my_state` is hardcoded to `"BOOT"` in `__init__`; `update_state()` exists but is never called by `mission_logic_node` (the nodes are separate processes with no direct reference)
  - Delta negotiation uses `known_mine_count` as a simple watermark — not a per-peer seq cursor. This can over-send duplicate beliefs.
  - No per-peer deduplication on the receive side; relies on BeliefStore merge logic to discard stale entries.
  - `add_local_mine()` public API exists but is never called (mine candidates flow via MineDelta from the task node, not directly into p2p_sync_node's store).

### mas_sync: belief_fusion
- **Implemented:**
  - Rule 1: higher seq wins (LWW)
  - Rule 2: equal seq → `confirmed` beats `rejected` (safety-first), then higher confidence
  - Rule 3: `confirmed` status is sticky — never downgraded by higher-seq `rejected` or lower-seq `candidate`
  - `get_delta_since(n)` returns all entries sorted by seq descending, sliced from index `n` (rough watermark approximation)
- **Missing:** No per-peer seq cursor; delta may re-send already-known entries.

### mas_task: p2p_task_node
- **Implemented:**
  - Mine candidate subscriber (`/{drone_id}/mine_candidates`) → announces `VERIFY_TAG` auction
  - Duplicate mine guard: `_resolved_mines` set skips re-announcing confirmed/rejected mines
  - Claim window: 3.0 s for mine candidates
  - `_tick()` resolves expired auctions, dispatches won tasks via `/{drone_id}/task_cmd` (JSON)
  - Abandoned task retry: tasks with no bidders are re-announced after 5 s
  - `busy` flag prevents bidding on low-priority tasks while executing one
- **Missing / Stub:**
  - `my_state` is set to `"BOOT"` and only updated via `update_state()`, which is never called from `mission_logic_node` (separate process)
  - `my_x`, `my_y` come from `/{drone_id}/pose` (PoseStamped), but `p2p_sync_node` now publishes PoseBeacon to `/team/pose_beacon` — the task node gets its own pose from the per-drone `/pose` topic which the explorer must publish
  - No `VERIFY_TAG` result publication from task node — the winning drone's explorer is expected to call `report_result()` directly, but there is no callback wiring from explorer → task node in the current integration

### mas_task: auction_manager
- **Implemented:**
  - Full open/claim/resolve lifecycle with `is_expired` property
  - Tie-breaking: lowest cost, then lexicographic drone_id
  - `pop_won_tasks()`, `pop_abandoned_tasks()`, `has_auction(task_id)`
  - `compute_cost()`: Euclidean distance / priority; returns `None` if `state in (FINISH, BOOT)` or `busy and priority < 0.8`
- **Missing:** Nothing critical. The `compute_cost` busy-penalty heuristic (adds 1000) is a placeholder.

### mas_mission: mission_logic_node
- **Implemented:**
  - Full arena parameters: `arena_width=91.44m`, `arena_height=24.38m`, `mission_duration=420s`
  - State machine + BT tick at 1 Hz
  - Own pose tracking from PoseBeacon (via `/team/pose_beacon`, including own drone)
  - Team dropout pruning: drones not heard from in > 5 s removed from `team_states`/`team_poses`
  - PATH_VERIFY: `BECOME_PATH_VERIFIER` auction, role assignment (verifier/explorer), AWAIT guard
  - CONVERGE: `BECOME_VERIFIER` auction, rescan announcements for `confidence < 0.7` candidates
  - `_task_registry` local dict mapping `task_id → task_type` (workaround for missing field in TaskResult)
  - Occupancy grid: full arena 91.44×24.38m at 0.5m/cell (182×48 cells), published once on CONVERGE entry
  - `mission_start` set on BOOT→SURVEY transition callback
- **Missing / Stub:**
  - `mission_logic_node` publishes `/{drone_id}/mission_cmd` (JSON) but nothing currently reads it — `stub_explorer.py` logs it, `ros2_adapter_v2.py` stores it in `_mission_cmd` but never acts on it
  - `_cmd_path_verify()` verifier waypoints use `self.sector` which is full arena corners (not a meaningful path corridor) — the actual 91.44m X-axis corridor is not encoded
  - p2p_sync_node `my_state` is never updated from mission_logic_node (separate process boundary)
  - `_all_drones_ready()` uses `team_last_seen` with 3 s threshold, so BOOT→SURVEY transition requires hearing from `num_drones - 1 = 3` others within 3 s — fragile at startup

### mas_mission: state_machine
- **Implemented:** All transitions, time thresholds (`T_PATH_VERIFY=90s`, `T_CONVERGE=45s`, `T_FINISH=10s`), transition callback registration.
- **Hardcoded:** Thresholds are module-level constants, not configurable at runtime.

### mas_mission: bt_runner
- **Implemented:** 6-node priority selector. Collision guard (`R_COLLISION=0.8m`), geofence guard (uses `node.arena_w/h`), failure monitor (pose stale > 3 s → FAILSAFE), task executor (consumes `pending_task_cmd`), exploration policy (dispatches per state), P2P sync manager (always SUCCESS).
- **Stub:** `P2PSyncManagerNode` is a no-op leaf — the actual sync protocol runs in a separate ROS2 process.

### ros2_adapter_v2 (waar_autonomy)
- **Implemented:**
  - Drives all 4 drones from a single `Ros2ExplorerNode` calling `BaselineLoop.tick()` at configurable Hz (default 2 Hz)
  - Manhattan-step movement toward assigned blocks; VERIFY_TAG task interrupt overrides exploration
  - Mine candidate publishing on `hazard_evidence >= 0.5` via `/{drone_id}/mine_candidates`
  - `_NullPerception` fallback when `SimPerceptionAdapter` import fails
  - Subscribes to `/{drone_id}/task_cmd` and `/{drone_id}/mission_cmd`
  - **PoseBeacon publishing:** `send_waypoint()` publishes both `/{drone_id}/pose` (PoseStamped) and `/team/pose_beacon` (PoseBeacon) so `p2p_sync_node` and `mission_logic_node` see real drone positions
  - **Spread start positions:** drones start at cols 11, 34, 57, 80 (world x = 11.5m, 34.5m, 57.5m, 80.5m) — over 22m separation, well clear of `R_COLLISION=0.8m`
  - **Position interpolation:** `send_waypoint()` advances at most `_NAV_MAX_STEP=2.0m` per call toward the target; prevents distance discontinuities that would thrash the neighbor graph
- **Known limitations:**
  - `beacon.state` is hardcoded to `"SURVEY"` — no sm_state bridge wired yet
  - `_on_mission_cmd` stores JSON in `_mission_cmd` but never acts on it — HOLD_POSITION, LAND_AND_SUBMIT, SWEEP_SECTOR are all ignored; BaselineLoop always runs its own planning
  - All 4 drones share one `WorldModel` instance — frontiers exhaust after ~1 drone covers the area, leaving the other 3 idle

---

## Known Bugs and Limitations

1. **p2p_sync_node my_state hardcoded to `"SURVEY"`:** The initial value was changed from `"BOOT"` to `"SURVEY"` to allow the BOOT→SURVEY transition to work (drones need to broadcast a non-BOOT state so `_all_drones_ready()` counts them). However, beacons now always show `"SURVEY"` regardless of actual state. Fix: add `/{drone_id}/sm_state` publisher in `mission_logic_node` and subscriber in `p2p_sync_node`.

2. **4 drones share one WorldModel → frontier exhaustion:** `ros2_adapter_v2.py` creates a single `WorldModel` shared by all 4 `BaselineLoop` instances. Once ~1 drone's worth of frontier is assigned, the other 3 have no frontiers left and stop moving. Fix: create one `WorldModel` per drone, or implement distance-based frontier assignment that reserves different regions.

3. **mission_cmd not acted on:** `ros2_adapter_v2.py` receives `mission_cmd` JSON (SWEEP_SECTOR, HOLD_POSITION, LAND_AND_SUBMIT, etc.) but discards it. BaselineLoop always runs its own planning regardless of the MAS coordinator's directives.

4. **TaskResult never published by explorer:** `p2p_task_node.report_result()` exists but nothing calls it. After winning a VERIFY_TAG task, the explorer has no mechanism to signal completion back to the task node. The `busy` flag never clears unless the explorer directly calls `report_result()`.

5. **Belief store in p2p_sync_node vs mine_beliefs in mission_logic_node are separate:** `p2p_sync_node.belief_store` is the authoritative fused store, but `mission_logic_node.mine_beliefs` is populated from `/team/mine_delta` directly. These two copies can diverge; mission decisions are made from the secondary copy.

6. **Delta watermark is imprecise:** `get_delta_since(n)` uses total count as watermark. If a mine belief is updated (seq incremented, same mine_id), the count doesn't change but the entry has new data — it will be missed by delta negotiation.

7. **PATH_VERIFY corridor waypoints are arena corners, not X-axis corridor:** `_cmd_path_verify()` sends the 4 corners of the full arena as waypoints. The IARC scoring expects a safe path along the X-axis (y ≈ arena_height/2).

8. **stub_explorer.py is not used in team_launch.py:** It was removed from the launch file. It must be started manually for standalone testing.

9. **`_all_drones_ready()` uses 3 s window:** At startup, all drones boot simultaneously. If any beacon is delayed by > 3 s, the drone stays in BOOT indefinitely.

---

## How to Run

### Docker (from ros2_ws/)
```bash
# Build image (run once or after package changes)
docker build -t mas_coordinator .

# Run with interactive terminal
docker run -it --rm \
  -v $(pwd)/src:/ros2_ws/src \
  mas_coordinator bash
```

### Build (inside container or sourced ROS2 env)
```bash
cd /ros2_ws
colcon build --packages-select mas_interfaces mas_sync mas_task mas_mission
source install/setup.bash
```

### Launch full 4-drone stack
```bash
ros2 launch mas_mission team_launch.py

# Override arena or timing parameters:
ros2 launch mas_mission team_launch.py \
  mission_duration:=420.0 \
  arena_width:=91.44 \
  arena_height:=24.38 \
  num_drones:=4
```

### Run stub_explorer (manual, for testing without ros2_adapter_v2)
```bash
# Start one stub per drone (each in its own terminal):
ros2 run mas_mission stub_explorer \
  --ros-args -p drone_id:=d1 -p sector_x_min:=0.0 -p sector_x_max:=91.44 \
             -p sector_y_min:=0.0 -p sector_y_max:=24.38

# Or all 4 at once using the old _SECTORS layout (22.86m wide lanes):
for i in 1 2 3 4; do
  ros2 run mas_mission stub_explorer --ros-args -p drone_id:=d$i &
done
```

### Run ros2_adapter_v2 (Kevin's integration)
```bash
# Requires waar_autonomy/src on PYTHONPATH
export PYTHONPATH=/path/to/waar_autonomy/src:$PYTHONPATH
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/waar_autonomy/src/adapters/ros2_adapter_v2.py
```

### Run tests (no ROS2 required)
```bash
cd /ros2_ws/src/mas_coordinator
python3 -m pytest tests/ -v

# Individual test files:
python3 -m pytest tests/test_auction_manager.py -v
python3 -m pytest tests/test_belief_fusion.py -v
python3 -m pytest tests/test_bt_runner.py -v
python3 -m pytest tests/test_state_machine.py -v
python3 -m pytest tests/test_edge_cases.py -v
```
Current result: **94 passed, 4 skipped** (skipped = removed `coord_to_xy` helper).

---

## Integration Points with Kevin's waar_autonomy

### What MAS publishes → waar_autonomy consumes
| Topic | Message | Publisher | Subscriber in waar_autonomy |
|---|---|---|---|
| `/{drone_id}/task_cmd` | `std_msgs/String` (JSON) | `p2p_task_node` | `ros2_adapter_v2._on_task_cmd()` |
| `/{drone_id}/mission_cmd` | `std_msgs/String` (JSON) | `mission_logic_node` | `ros2_adapter_v2._on_mission_cmd()` (stored, not acted on) |

### What waar_autonomy publishes → MAS consumes
| Topic | Message | Publisher | Subscriber in MAS |
|---|---|---|---|
| `/{drone_id}/pose` | `geometry_msgs/PoseStamped` | `ros2_adapter_v2` | `p2p_sync_node._on_local_pose()`, `p2p_task_node._on_local_pose()` |
| `/team/pose_beacon` | `mas_interfaces/PoseBeacon` | **NOT YET PUBLISHED** (fix reverted) | `p2p_sync_node._on_pose_beacon()`, `mission_logic_node._on_pose_beacon()` |
| `/{drone_id}/mine_candidates` | `mas_interfaces/MineBelief` | `ros2_adapter_v2.Ros2PerceptionAdapter` | `p2p_task_node._on_mine_candidate()` |

### What still needs to be connected
1. **`/team/pose_beacon` publishing** — re-apply the reverted fix in `ros2_adapter_v2.py` so `Ros2NavigationAdapter.send_waypoint()` publishes a `PoseBeacon` to `/team/pose_beacon` after each position update.

2. **`/{drone_id}/sm_state` bridge** — `mission_logic_node` should publish its current state as `std_msgs/String` on `/{drone_id}/sm_state`; `p2p_sync_node` should subscribe to update `my_state` so beacons carry the correct state string.

3. **Task result reporting** — after a VERIFY_TAG task completes, `ros2_adapter_v2` must call `p2p_task_node.report_result()` or publish a `TaskResult` to `/team/task_result` so the `busy` flag clears and mine beliefs are updated.

4. **Mission cmd actuation** — `ros2_adapter_v2._on_mission_cmd()` currently discards directives. At minimum, `HOLD_POSITION` and `LAND_AND_SUBMIT` should pause/stop the `BaselineLoop`.

5. **Start position spread + interpolation** — re-apply the two reverted fixes (evenly spread drone start cols, `_NAV_MAX_STEP=2.0m` interpolation) to eliminate startup collisions and neighbor graph thrashing.

---

## Next Steps (Prioritised)

1. **Wire sm_state bridge** — add `/{drone_id}/sm_state` publisher (`std_msgs/String`) in `mission_logic_node._on_state_transition()` and subscriber in `p2p_sync_node` updating `self.my_state`; needed for accurate team state in beacons and convergence checks.

2. **Fix frontier exhaustion** — either create one `WorldModel` per drone in `ros2_adapter_v2.py`, or add distance-based frontier assignment that reserves different arena strips for each drone; without this 3 of 4 drones are effectively idle.

3. **Wire task result reporting** — `ros2_adapter_v2` must publish `TaskResult` to `/team/task_result` when a VERIFY_TAG task completes; needed for mine belief updates, `busy` flag clearing, and auction registry cleanup.

4. **Implement mission_cmd actuation** — `ros2_adapter_v2._on_mission_cmd()` currently discards all directives; at minimum handle `HOLD_POSITION` (pause `BaselineLoop`) and `LAND_AND_SUBMIT` (stop node).

5. **Add network delay simulation** — all sync and auction messages fire instantly in the current setup; add configurable per-message latency to test robustness of the auction timing and sync window protocol.

6. **Fix PATH_VERIFY corridor waypoints** — `_cmd_path_verify()` sends the 4 corners of the full arena as waypoints; should send a straight X-axis corridor (`[[0, arena_h/2], [arena_w, arena_h/2]]`) matching IARC scoring expectations.

7. **Fix delta watermark** — replace `get_delta_since(count)` with per-peer seq cursors in `BeliefStore` to avoid missing updated (not new) beliefs.

8. **Fix `_all_drones_ready()` race at startup** — increase the stale threshold to 10 s for the first BOOT→SURVEY check, or add a configurable `boot_timeout` parameter.

---

## Integration History

Summary of what was done to get the MAS stack and waar_autonomy talking to each other:

- **`ros2_adapter_v2.py` created** to bridge Kevin's `BaselineLoop` (waar_autonomy) to ROS2 topics; wraps `NavigationPort` and `PerceptionPort`, drives all 4 drones from one `Ros2ExplorerNode`.

- **PoseBeacon publishing added** to `Ros2NavigationAdapter.send_waypoint()` — publishes `/team/pose_beacon` (in addition to `/{drone_id}/pose`) on every position update so `p2p_sync_node` and `mission_logic_node` receive real drone positions; without this the neighbor graph was permanently empty.

- **Drone start positions spread** across arena start edge — cols 11, 34, 57, 80 (≈22m apart) replacing cols 0, 1, 2, 3 (1m apart) which triggered constant COLLISION warnings at boot.

- **Position interpolation added** (`_NAV_MAX_STEP=2.0m` per call) — prevents `send_waypoint()` from jumping instantly to a waypoint; keeps published distances smooth so `p2p_sync_node` hysteresis (`r_enter=8m`, `r_exit=12m`) doesn't thrash.

- **`p2p_sync_node.my_state` initialised to `"SURVEY"`** (was `"BOOT"`) — `mission_logic_node._all_drones_ready()` counts drones whose last-seen PoseBeacon is within 3 s; if `my_state == "BOOT"` the beacon was still ignored in earlier logic, blocking the BOOT→SURVEY transition.

- **`R_COLLISION` reduced from 2.0m to 0.8m** — BaselineLoop assigns drones to adjacent 1m blocks so 1m separation is expected and safe; the original 2m threshold triggered constant HOLD commands during normal exploration.

- **`stub_explorer.py` removed from `team_launch.py`** — replaced by `ros2_adapter_v2.py` as the authoritative explorer; stub remains in the package for standalone MAS testing without waar_autonomy.

- **Full end-to-end mission verified**: BOOT → SURVEY → PATH_VERIFY → CONVERGE → FINISH with `team_launch.py` + `ros2_adapter_v2.py` running together and real PoseBeacon positions flowing through the neighbor graph.
