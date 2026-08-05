# Multi-Agent Coordination System — IARC Mission 10

Four drones coordinate to find and verify mines in a 91.44 m x 24.38 m arena (300 ft x 80 ft) within 7 minutes. Every drone runs three ROS2 nodes (sync, task, mission) using a fully decentralised architecture — no central server, no shared clock. A separate adapter bridges Kevin's `waar_autonomy` BaselineLoop planner to the MAS topics.

---

## Architecture

```
+-------------------------------------------------------------------------------+
|                          Mission Layer  (per drone)                           |
|                                                                               |
|   State Machine: BOOT -> SURVEY -> VERIFY_TAG -> PATH_VERIFY -> CONVERGE -> FINISH
|                                                                               |
|   Behavior Tree (priority selector):                                          |
|     1) CollisionGuard   (R=0.8m)                                              |
|     2) GeofenceGuard    (arena bounds)                                        |
|     3) FailureMonitor   (pose stale > 3s -> FAILSAFE)                         |
|     4) TaskExecutor     (consume pending task_cmd)                            |
|     5) ExplorationPolicy (dispatch per-state mission cmd)                     |
|     6) P2PSyncManager   (always SUCCESS, sync runs in separate process)       |
+-------------------------------------------------------------------------------+
          ^                       ^                       ^
          | task_cmd (JSON)       | mine_delta            | sm_state (TODO)
          |                       |                       |
+-------------------------------------------------------------------------------+
|                       Coordination Layer  (fully P2P)                        |
|                                                                               |
|   p2p_sync_node:  pose beacons -> neighbor graph -> sync window -> MineDelta  |
|   p2p_task_node:  mine candidate -> announce -> claim -> resolve -> task_cmd  |
|                                                                               |
|   Shared topics:  /team/pose_beacon  /team/sync_hello  /team/sync_ack        |
|                   /team/mine_delta   /team/task_announce  /team/task_claim    |
|                   /team/task_result                                           |
+-------------------------------------------------------------------------------+
          ^                       ^
          | /{drone_id}/pose      | /{drone_id}/mine_candidates
          | /{drone_id}/task_cmd  | /{drone_id}/mission_cmd
          |                       |
+-------------------------------------------------------------------------------+
|                    Autonomy Layer  (waar_autonomy, Kevin)                     |
|                                                                               |
|   ros2_adapter_v2.py:  Ros2ExplorerNode drives BaselineLoop for all 4 drones |
|     - Publishes /{drone_id}/pose (PoseStamped) + /team/pose_beacon (PoseBeacon)
|     - Publishes /{drone_id}/mine_candidates on hazard_evidence >= 0.5        |
|     - Subscribes /{drone_id}/task_cmd  (VERIFY_TAG interrupts exploration)   |
|     - Subscribes /{drone_id}/mission_cmd  (stored, not yet acted on)         |
+-------------------------------------------------------------------------------+
```

---

## Packages

| Package | Type | Responsibility |
|---|---|---|
| `mas_interfaces` | ament_cmake | 8 custom message definitions |
| `mas_sync` | ament_python | Beacon broadcast, neighbor graph, Scheme-B sync window, belief fusion |
| `mas_task` | ament_python | P2P task auction (announce / claim / resolve / retry) |
| `mas_mission` | ament_python | State machine, BT priority selector, mission directives, occupancy grid |

---

## ROS2 Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/team/pose_beacon` | `PoseBeacon` | `p2p_sync_node`, `ros2_adapter_v2` | `p2p_sync_node`, `mission_logic_node` |
| `/team/sync_hello` | `SyncHello` | `p2p_sync_node` | `p2p_sync_node` |
| `/team/sync_ack` | `SyncAck` | `p2p_sync_node` | `p2p_sync_node` |
| `/team/mine_delta` | `MineDelta` | `p2p_sync_node` | `p2p_sync_node`, `mission_logic_node` |
| `/team/task_announce` | `TaskAnnounce` | `p2p_task_node`, `mission_logic_node` | `p2p_task_node` |
| `/team/task_claim` | `TaskClaim` | `p2p_task_node` | `p2p_task_node` |
| `/team/task_result` | `TaskResult` | explorer (TODO) | `p2p_task_node`, `mission_logic_node` |
| `/{drone_id}/pose` | `PoseStamped` | `ros2_adapter_v2` | `p2p_sync_node`, `p2p_task_node` |
| `/{drone_id}/mine_candidates` | `MineBelief` | `ros2_adapter_v2` | `p2p_task_node` |
| `/{drone_id}/task_cmd` | `String` (JSON) | `p2p_task_node` | `ros2_adapter_v2` |
| `/{drone_id}/mission_cmd` | `String` (JSON) | `mission_logic_node` | `ros2_adapter_v2` |
| `/{drone_id}/safe_path_grid` | `OccupancyGrid` | `mission_logic_node` | scoring system |

---

## Quick Start

### Docker
```bash
# Start existing container
docker start mas_dev
docker exec -it mas_dev bash

# Source ROS2 (or add to ~/.bashrc)
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

### Build
```bash
cd /ros2_ws
colcon build --packages-select mas_interfaces
source install/setup.bash
colcon build --packages-select mas_sync mas_task mas_mission
source install/setup.bash
```

### Launch MAS (4-drone coordinator stack)
```bash
ros2 launch mas_mission team_launch.py
# Optional overrides:
ros2 launch mas_mission team_launch.py mission_duration:=420.0 arena_width:=91.44 arena_height:=24.38
```

### Launch explorer (Kevin's waar_autonomy)
```bash
export PYTHONPATH=/path/to/waar_autonomy/src:$PYTHONPATH
python3 waar_autonomy/src/adapters/ros2_adapter_v2.py
```

### Monitor mission state
```bash
python3 -c "
import rclpy, sys
from rclpy.node import Node
from mas_interfaces.msg import PoseBeacon
class M(Node):
    def __init__(self):
        super().__init__('monitor')
        self.create_subscription(PoseBeacon, '/team/pose_beacon', self.cb, 20)
    def cb(self, msg):
        print(f'{msg.drone_id:4s}  ({msg.x:6.1f},{msg.y:6.1f})  {msg.state}')
rclpy.init(); rclpy.spin(M())
"
```

### Tests (no ROS2 required)
```bash
cd /ros2_ws/src/mas_coordinator
python3 -m pytest tests/ -v
# Expected: 94 passed, 4 skipped
```

---

## Current Status

### Works
- Full mission state machine BOOT -> SURVEY -> PATH_VERIFY -> CONVERGE -> FINISH
- P2P belief sync with Scheme-B sync windows (hello/ack/delta)
- Mine belief fusion: seq-LWW, confirmed-sticky, confirmed beats rejected on equal seq
- Distributed task auction with no-bidder retry and duplicate mine guard
- 6-layer BT priority selector with collision, geofence, and failure guards
- Integration with waar_autonomy BaselineLoop via `ros2_adapter_v2.py`
- PoseBeacon published to `/team/pose_beacon` so MAS sees real drone positions
- 94 unit tests passing

### Known Issues
- Drone state in PoseBeacon is hardcoded to `"SURVEY"` — sm_state bridge not wired
- All 4 drones share one WorldModel in ros2_adapter_v2; frontiers exhaust after ~1 drone explores the area, leaving the other 3 idle
- Task result not reported back from explorer — `busy` flag in p2p_task_node never clears
- `mission_cmd` (HOLD_POSITION, LAND_AND_SUBMIT, SWEEP_SECTOR) received but not acted on by ros2_adapter_v2
- Network delay not simulated — sync windows fire instantly

---

## File Structure

```
mas_interfaces/
  msg/MineBelief.msg        mine_id, x, y, confidence, status, seq
  msg/PoseBeacon.msg        drone_id, x, y, z, heading_deg, state, battery_pct
  msg/MineDelta.msg         batch MineBelief updates with TTL relay
  msg/SyncHello.msg         initiates sync window, carries known_mine_count
  msg/SyncAck.msg           acknowledges SyncHello
  msg/TaskAnnounce.msg      task_id, task_type, target_x/y, priority, claim_window_s
  msg/TaskClaim.msg         bidder bid with cost
  msg/TaskResult.msg        executor outcome, mine_id, confidence

mas_sync/
  p2p_sync_node.py          beacon, neighbor graph, sync window, MineDelta relay
  belief_fusion.py          BeliefStore: seq-LWW merge with sticky confirmed status

mas_task/
  p2p_task_node.py          mine candidate -> auction -> task_cmd dispatch, retry
  auction_manager.py        AuctionEntry, AuctionManager, compute_cost()

mas_mission/
  state_machine.py          StateMachine + MissionContext (pure Python, no ROS2)
  bt_runner.py              PrioritySelector + 6 BT nodes (pure Python, no ROS2)
  mission_logic_node.py     ROS2 node: owns SM + BT, PATH_VERIFY role split, occupancy grid
  stub_explorer.py          Integration test stub: lawnmower pose, one mine after 30s
  launch/team_launch.py     3 nodes x 4 drones; arena/duration args

tests/
  test_auction_manager.py   AuctionManager and compute_cost unit tests
  test_belief_fusion.py     BeliefStore merge rule unit tests
  test_bt_runner.py         All 6 BT nodes via Mock (no ROS2)
  test_state_machine.py     All StateMachine transition unit tests
  test_edge_cases.py        Arena dims, 420s, no-bidder retry, dropout, belief conflict

waar_autonomy/src/adapters/
  ros2_adapter_v2.py        Bridges BaselineLoop to ROS2: pose, beacons, mines, task cmds
```
