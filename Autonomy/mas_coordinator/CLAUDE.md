# MAS Coordinator — CLAUDE.md

## Project Overview

Multi-agent drone coordination system for IARC Mission 10 (300 ft × 80 ft arena, 7-minute mission, 4 drones). Each drone runs three independent ROS2 nodes: `p2p_sync_node` (belief synchronisation via a Scheme-B sync-window protocol), `p2p_task_node` (distributed task auction), and `mission_logic_node` (state machine + behaviour tree). A fourth component, `ros2_adapter_v2.py`, lives in `waar_autonomy/src/adapters/` and bridges the MAS topics to a planner. All four MAS nodes are pure-Python, use only `mas_interfaces` custom messages, and are designed to run without a centralised coordinator.

> **Correction (see "ros2_adapter_v2" below):** earlier drafts of this doc described `ros2_adapter_v2.py` as bridging a class called `BaselineLoop`. That class does not exist anywhere in this repo, and neither did the adapter file itself — despite the "Integration History" section below narrating a full end-to-end integration as if it had already happened. It has now been implemented, but against the planner that actually exists in this repo: `application.simulator.Simulator`. Treat everything below tagged **[as-implemented]** as accurate to the real code; everything else in this file describes the original plan, which does not fully match what `Simulator` can do (see "Known limitations" under ros2_adapter_v2).

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

### ros2_adapter_v2 (waar_autonomy) — [as-implemented]
Lives at `waar_autonomy/src/adapters/ros2_adapter_v2.py`. Bridges MAS topics to `application.simulator.Simulator` (WorldModel + DroneState + GroundTruthPort, ticked through observe → corridor → certify → frontier) — **not** `BaselineLoop`, which does not exist in this repo.

- **Implemented:**
  - Single `Ros2ExplorerNode` drives all drones (`drone_ids` param, default `d1..d4`), each with its **own** `Simulator`/`WorldModel`/`DroneState`/ground-truth map — built correctly from the start, not shared
  - Ticks each drone's `Simulator.tick()` at configurable Hz (`tick_hz` param, default 2 Hz), unless held or executing a task
  - `VERIFY_TAG` task interrupt: on `/{drone_id}/task_cmd`, steps the drone one block per tick toward `(target_x, target_y)` (converted via `xy_to_coord`) instead of exploring, calling `observe_block()` along the way so the world model stays current
  - On reaching a `VERIFY_TAG` target, publishes `TaskResult` directly to `/team/task_result` (outcome `"confirmed"`, confidence `1.0`) — `p2p_task_node` runs in a separate process, so this goes over the topic, not a direct method call
  - `/{drone_id}/mission_cmd`: `HOLD_POSITION` pauses ticking, `LAND_AND_SUBMIT` stops the drone permanently; all other commands (`SWEEP_SECTOR`, `FILL_GAPS`, `VERIFY_PATH`, `STANDBY_FOR_TASK`, `AWAIT_PATH_VERIFY`) are logged only
  - Publishes `/{drone_id}/pose` (PoseStamped) and `/team/pose_beacon` (PoseBeacon) every tick from `Simulator.drone.block`
  - Publishes `/{drone_id}/mine_candidates` for each newly-detected `HAZARD` fine cell in `WorldModel.detected`, confidence `1.0` (see "Known limitations" — this isn't a continuous score)
  - `coord_to_xy` / `xy_to_coord` pure functions match the contract `mas_coordinator/tests/test_state_machine.py::TestCoordConversion` already expected
- **Known limitations:**
  - **`Simulator` ≠ `BaselineLoop`:** no continuous XY position, no velocity/interpolation, no built-in collision avoidance between drones — `DroneState.block` teleports one block per tick. The previously-documented "spread start positions" / "position interpolation" fixes don't apply to this architecture; each drone's block grid is independent so there's no literal collision to avoid in the first place.
  - **Arena scale mismatch:** `coord_to_xy` uses `CELL_SIZE=1.0` as a placeholder. `waar_autonomy`'s block grid has no inherent physical scale (`Config` defaults to a 20×15-block test arena), while `mission_logic_node` assumes a real 91.44m × 24.38m arena. A fully-explored simulated arena only ever reports as a ~20m × 15m box — `mission_logic_node`'s geofence/collision guards, sized for the real arena, won't see meaningful bounds against these poses.
  - `beacon.state` is a local guess (`"VERIFY_TAG"` while a task is active, else `"SURVEY"`) — no sm_state bridge wired yet, so this still isn't the authoritative `mission_logic_node` state
  - Binary hazard detection, not a confidence score: `Simulator`'s sensing model only has `HAZARD`/not-`HAZARD`, unlike the continuous `hazard_evidence` this doc previously assumed. Every newly-detected hazard cell is reported once at confidence `1.0`.
  - Sector-constrained exploration (`SWEEP_SECTOR`, `FILL_GAPS`) isn't wired in — `Simulator`'s frontier scorer (`best_frontier`) has no concept of a bounded sector, so these directives are logged and ignored
  - `PATH_VERIFY`-specific directives (`VERIFY_PATH` waypoints, `BECOME_PATH_VERIFIER`) aren't handled — only `VERIFY_TAG` task_cmds trigger the target-seeking behavior above

---

## Known Bugs and Limitations

1. **p2p_sync_node my_state hardcoded to `"SURVEY"`:** The initial value was changed from `"BOOT"` to `"SURVEY"` to allow the BOOT→SURVEY transition to work (drones need to broadcast a non-BOOT state so `_all_drones_ready()` counts them). However, beacons now always show `"SURVEY"` regardless of actual state. Fix: add `/{drone_id}/sm_state` publisher in `mission_logic_node` and subscriber in `p2p_sync_node`.

2. ~~4 drones share one WorldModel → frontier exhaustion~~ **Fixed [as-implemented]:** `ros2_adapter_v2.py` now builds one `WorldModel`/`DroneState`/ground-truth map per drone. Note this doesn't add sector-based deconfliction — with no bounded exploration, independent drones can still cover overlapping ground, just not by sharing a single instance's state anymore.

3. **mission_cmd only partially acted on [as-implemented, partial]:** `HOLD_POSITION` (pause) and `LAND_AND_SUBMIT` (stop) are now actuated. `SWEEP_SECTOR`, `FILL_GAPS`, `VERIFY_PATH`, `STANDBY_FOR_TASK`, and `AWAIT_PATH_VERIFY` are still received but discarded — `Simulator`'s frontier scorer has no sector-bounded exploration mode to hand these to.

4. ~~TaskResult never published by explorer~~ **Fixed for VERIFY_TAG [as-implemented]:** `ros2_adapter_v2.py` now publishes `TaskResult` directly to `/team/task_result` when a drone reaches a `VERIFY_TAG` target (it can't call `p2p_task_node.report_result()` as a method — that node is a separate process — so it publishes the topic instead). This only covers `VERIFY_TAG`; `BECOME_PATH_VERIFIER` / `BECOME_VERIFIER` task types still get no result report from the adapter.

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

## Integration Points with waar_autonomy

### What MAS publishes → waar_autonomy consumes
| Topic | Message | Publisher | Subscriber in waar_autonomy |
|---|---|---|---|
| `/{drone_id}/task_cmd` | `std_msgs/String` (JSON) | `p2p_task_node` | `ros2_adapter_v2._make_task_cmd_handler()` |
| `/{drone_id}/mission_cmd` | `std_msgs/String` (JSON) | `mission_logic_node` | `ros2_adapter_v2._make_mission_cmd_handler()` (`HOLD_POSITION`/`LAND_AND_SUBMIT` acted on; rest logged only) |

### What waar_autonomy publishes → MAS consumes
| Topic | Message | Publisher | Subscriber in MAS |
|---|---|---|---|
| `/{drone_id}/pose` | `geometry_msgs/PoseStamped` | `ros2_adapter_v2._publish_pose()` | `p2p_sync_node._on_local_pose()`, `p2p_task_node._on_local_pose()` |
| `/team/pose_beacon` | `mas_interfaces/PoseBeacon` | `ros2_adapter_v2._publish_pose()` | `p2p_sync_node._on_pose_beacon()`, `mission_logic_node._on_pose_beacon()` |
| `/{drone_id}/mine_candidates` | `mas_interfaces/MineBelief` | `ros2_adapter_v2._publish_new_mines()` | `p2p_task_node._on_mine_candidate()` |
| `/team/task_result` | `mas_interfaces/TaskResult` | `ros2_adapter_v2._report_task_result()` (VERIFY_TAG only) | `p2p_task_node._on_task_result()`, `mission_logic_node._on_task_result()` |

### What still needs to be connected
1. ~~`/team/pose_beacon` publishing~~ **Done [as-implemented]**.

2. **`/{drone_id}/sm_state` bridge** — `mission_logic_node` should publish its current state as `std_msgs/String` on `/{drone_id}/sm_state`; `p2p_sync_node` should subscribe to update `my_state`, and `ros2_adapter_v2` should subscribe too so `PoseBeacon.state` reflects the real state machine instead of a local guess.

3. ~~Task result reporting~~ **Done for VERIFY_TAG [as-implemented]** — `ros2_adapter_v2._report_task_result()` publishes `TaskResult` directly (it can't call `p2p_task_node.report_result()`, a separate process). `BECOME_PATH_VERIFIER`/`BECOME_VERIFIER` tasks still get no result.

4. ~~Mission cmd actuation~~ **Minimum bar done [as-implemented]** — `HOLD_POSITION` and `LAND_AND_SUBMIT` are handled. `SWEEP_SECTOR`/`FILL_GAPS`/`VERIFY_PATH` need a sector-aware exploration mode in `Simulator` that doesn't exist yet; wiring the JSON through is not enough by itself.

5. ~~Start position spread + interpolation~~ **Not applicable to the current adapter** — `Simulator.DroneState` teleports one block per tick with no continuous position or velocity, so there's nothing to interpolate and no shared-space collision to spread apart (each drone has its own grid). This was written for the documented `BaselineLoop`-based design, which doesn't exist in this repo.

6. **Arena scale reconciliation** — `coord_to_xy`'s `CELL_SIZE=1.0` placeholder means simulated block coordinates and `mission_logic_node`'s real 91.44m×24.38m arena assumptions don't actually line up. Needs an explicit decision (scale `CELL_SIZE` to match, resize the block grid, or something else) before geofence/collision guards mean anything against these poses.

---

## Next Steps (Prioritised)

1. **Wire sm_state bridge** — add `/{drone_id}/sm_state` publisher (`std_msgs/String`) in `mission_logic_node._on_state_transition()` and subscribers in `p2p_sync_node` (updating `self.my_state`) and `ros2_adapter_v2` (updating `PoseBeacon.state` instead of guessing); needed for accurate team state in beacons and convergence checks.

2. ~~Fix frontier exhaustion~~ **Done [as-implemented]** — `ros2_adapter_v2.py` builds one `WorldModel` per drone.

3. ~~Wire task result reporting~~ **Done for VERIFY_TAG [as-implemented]**.

4. ~~Implement mission_cmd actuation~~ **Minimum bar done [as-implemented]** — `HOLD_POSITION`/`LAND_AND_SUBMIT` handled; sector-based directives still need a `Simulator`-side exploration mode before they can be actuated (see "Known limitations" above).

5. **Add network delay simulation** — all sync and auction messages fire instantly in the current setup; add configurable per-message latency to test robustness of the auction timing and sync window protocol.

6. **Fix PATH_VERIFY corridor waypoints** — `_cmd_path_verify()` sends the 4 corners of the full arena as waypoints; should send a straight X-axis corridor (`[[0, arena_h/2], [arena_w, arena_h/2]]`) matching IARC scoring expectations. Note `ros2_adapter_v2` doesn't act on `VERIFY_PATH` task_cmds at all yet, so this is blocked on more than just the waypoint shape.

7. **Fix delta watermark** — replace `get_delta_since(count)` with per-peer seq cursors in `BeliefStore` to avoid missing updated (not new) beliefs.

8. **Fix `_all_drones_ready()` race at startup** — increase the stale threshold to 10 s for the first BOOT→SURVEY check, or add a configurable `boot_timeout` parameter.

9. **Reconcile arena scale** — `ros2_adapter_v2`'s `CELL_SIZE=1.0` placeholder doesn't match `mission_logic_node`'s real 91.44m×24.38m arena assumption. Needs an explicit decision before geofence/collision guards are meaningful.

---

## Integration History

The section below described a full end-to-end integration (PoseBeacon publishing, spread start positions, interpolation, a verified BOOT→FINISH run) as something that had already happened, written against a `BaselineLoop`-based adapter. Neither that adapter nor `BaselineLoop` itself ever existed in this repo — there is no commit touching `ros2_adapter_v2.py` in this repo's git history prior to it being written against `Simulator` instead. Treat the bullets below as the original design intent, not as things that happened here.

- ~~`ros2_adapter_v2.py` created~~ to bridge *Kevin's `BaselineLoop`* (waar_autonomy) to ROS2 topics — described, never built. What exists now bridges `application.simulator.Simulator` instead (see "ros2_adapter_v2 (waar_autonomy) — [as-implemented]" above), which has no `NavigationPort`/`PerceptionPort` wrapping, no continuous XY position, and no built-in collision avoidance.

- PoseBeacon publishing, start-position spread, and position interpolation as described below were written for the `BaselineLoop` design and do not apply to `Simulator`'s block-teleport movement model. PoseBeacon publishing *is* implemented in the current adapter (see "What waar_autonomy publishes → MAS consumes" above); the other two aren't meaningful for this architecture.

- `p2p_sync_node.my_state` initialised to `"SURVEY"`, `R_COLLISION` reduced to 0.8m, and `stub_explorer.py` removed from `team_launch.py` — these are changes to the MAS-side packages themselves, independent of which explorer/adapter is running, and remain accurate.

- The "full end-to-end mission verified" claim below has not been re-verified against the current `ros2_adapter_v2.py` — it was never run, since the file didn't exist. Verifying `team_launch.py` + the current `ros2_adapter_v2.py` together is still open work.

Original (unverified against this repo) narrative, preserved for context:

- **PoseBeacon publishing added** to `Ros2NavigationAdapter.send_waypoint()` — publishes `/team/pose_beacon` (in addition to `/{drone_id}/pose`) on every position update so `p2p_sync_node` and `mission_logic_node` receive real drone positions; without this the neighbor graph was permanently empty.

- **Drone start positions spread** across arena start edge — cols 11, 34, 57, 80 (≈22m apart) replacing cols 0, 1, 2, 3 (1m apart) which triggered constant COLLISION warnings at boot.

- **Position interpolation added** (`_NAV_MAX_STEP=2.0m` per call) — prevents `send_waypoint()` from jumping instantly to a waypoint; keeps published distances smooth so `p2p_sync_node` hysteresis (`r_enter=8m`, `r_exit=12m`) doesn't thrash.

- **`p2p_sync_node.my_state` initialised to `"SURVEY"`** (was `"BOOT"`) — `mission_logic_node._all_drones_ready()` counts drones whose last-seen PoseBeacon is within 3 s; if `my_state == "BOOT"` the beacon was still ignored in earlier logic, blocking the BOOT→SURVEY transition.

- **`R_COLLISION` reduced from 2.0m to 0.8m** — BaselineLoop assigns drones to adjacent 1m blocks so 1m separation is expected and safe; the original 2m threshold triggered constant HOLD commands during normal exploration.

- **`stub_explorer.py` removed from `team_launch.py`** — replaced by `ros2_adapter_v2.py` as the authoritative explorer; stub remains in the package for standalone MAS testing without waar_autonomy.

- **Full end-to-end mission verified**: BOOT → SURVEY → PATH_VERIFY → CONVERGE → FINISH with `team_launch.py` + `ros2_adapter_v2.py` running together and real PoseBeacon positions flowing through the neighbor graph.
