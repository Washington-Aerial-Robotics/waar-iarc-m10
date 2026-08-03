# A) For Each drone (×4) 


### 1. `explorer_node` (Kevin - Skipp what he did here)
---

### 2. `p2p_sync_node` 

* Implements **Scheme B sync window**: neighbor graph + hello/ack + missing delta exchange

* Performs **belief fusion**:

  * mine belief clustering / confidence update
  * (optional) tile uncertainty fusion

* **Subscribes/Publishes:**

```
/team/pose_beacon
/team/sync_hello
/team/sync_ack
/team/mine_delta
(optional) /team/tile_delta
```

* (optional) publishes locally:

```
/drone_i/local_belief
```

---

### 3. `p2p_task_node` 

* Fully **P2P task auction system**

Components:

* announce
* claim
* result
* claim window

The **winner sends `task_cmd` to the local explorer** (interrupting exploration).

* **Subscribes/Publishes:**

```
/team/task_announce
/team/task_claim
/team/task_result
```

* **Local subscriptions**

```
/drone_i/mine_candidates
/drone_i/status
```

---

### 4. `mission_logic_node`

(merge it into `p2p_task_node` or independent)

Responsibilities:

* Runs the **State Machine + Behavior Tree runner**
  (each drone runs its own copy)
* Manages:

  * timers
  * geofence
  * time thresholds
  * state transitions
* Sends `task_cmd` to the local explorer
* (optional) broadcasts `mode_beacon` for team observation/consistency

---

# B) State Machine Definition

```
BOOT → SURVEY → VERIFY_TAG → PATH_VERIFY → CONVERGE → FINISH
```

For each state we define:

* **Entry condition**
* **Exit condition / transition**
* **Primary objective**
* **Possible P2P task types**

---

# 1) BOOT

### Objective

System initialization: network ready, localization ready.

### Entry

* System startup automatically enters **BOOT**

### Actions

(Not part of the main BT — usually an initialization sequence)

* Initialize ROS2 publishers/subscribers
* Initialize FastDDS domain
* Start publishing `pose_beacon`
  (even if still at origin)
* Create neighbor table (can start empty)
* Start sync node (able to receive hello/ack)

### Exit → SURVEY

Conditions:

```
pose_valid == true
explorer_node_ready == true
time_since_start < timeout_boot
```

Optional:

```
at least N_min_neighbors pose_beacon detected
```

(In simulation this requirement can be skipped)

---

# 2) SURVEY

### Objective

Maximize **coverage**, accumulate **mine beliefs**, maintain **team safety**, and keep **P2P synchronization active**.

### Entry

* From **BOOT**
* Or return from **VERIFY_TAG**

### Exit

```
→ VERIFY_TAG
    if local drone wins a verify task auction

→ PATH_VERIFY
    if remaining_time < T_path_verify_start (recommended: 120 s)

→ CONVERGE
    if remaining_time < T_converge_start (recommended: 90 s)

→ FINISH
    if remaining_time < T_finish (e.g., 5–10 s)

→ FAILSAFE (optional)
    severe risk (unsolvable collision / control failure)
```

### Possible P2P Tasks During SURVEY

* `VERIFY_TAG` (mine candidate verification)

Optional:

* `YIELD` (collision avoidance)
  but usually handled locally by the safety layer.

---

# 3) VERIFY_TAG

### Objective

Perform **close-range verification of a mine candidate**
(e.g., AprilTag / close inspection).

### Entry

* Drone wins `VERIFY_TAG(task_id)` during claim window

OR

* Local detection with deterministic self-assignment
  (not recommended — causes duplicate verification)

### Exit

```
→ SURVEY
    task completed (confirmed / rejected / uncertain)

→ CONVERGE / PATH_VERIFY
    if time thresholds are reached

→ FAILSAFE
    geofence violation or unresolved collision
```

### Task Completion Conditions

* Reach target radius `r_verify` (e.g., 1–2 m)
* Stay for `t_dwell` (e.g., 2–5 s)
* Acquire tag evidence or fail
* Publish `/team/task_result`

---

# 4) PATH_VERIFY

### Objective

Before entering **CONVERGE**, reduce uncertainty along the **safe path corridor**:

* narrow segments
* low-confidence mines
* path buffer regions

### Entry

```
remaining_time < T_path_verify_start (recommended: 120 s)
```

or (advanced trigger)

```
W_min < threshold
uncertainty_along_path high
```

### Exit

```
→ CONVERGE
    remaining_time < T_converge_start (recommended: 90 s)

→ VERIFY_TAG
    high-priority mine appears

→ FINISH
    time limit reached
```

### P2P Tasks in PATH_VERIFY

* `BECOME_PATH_VERIFIER`
* `VERIFY_PATH_SEGMENT`
* `RESCAN_PATH_BUFFER`

**MVP**

Select **1 drone as Path Verifier**,
while the other **3 continue exploration / gap filling**.

---

# 5) CONVERGE

### Objective

Final convergence phase:

* align **team beliefs**
* rescan **critical areas**
* final verification before output

### Entry

```
remaining_time < T_converge_start (recommended: 90 s)
```

### Exit → FINISH

```
remaining_time < T_finish
```

### Possible P2P Tasks

* `BECOME_VERIFIER`
* `VERIFY_TAG` (only for mines affecting path)
* `RESCAN` (critical path segments / buffers)

---

# 6) FINISH

### Objective

Stop new tasks and **freeze final outputs**.

### Entry

```
remaining_time < T_finish
```

### Actions

* Stop new `task_announce`
* Explorer enters **hold/hover**
* Publish final summaries:

  * mine beliefs
  * safe path (if available)

### Exit

Competition ends → node shutdown.

---

# C) Behavior Tree (BT) Structure Per State

**Key principle**

The **BT skeleton remains the same**, but different states modify:

* **Task Executor goals**
* **Exploration policy weight**

---

# Shared BT Skeleton (All States)

Priority Selector (top = highest priority)

1. **Safety / Collision Guard**
   local collision avoidance and speed limits

2. **Geofence + Time Guard**
   pull back from boundary, trigger state transitions

3. **Failure Monitor**
   detect perception/control anomalies

4. **Task Executor**
   execute assigned / won tasks

5. **Exploration Policy**
   frontier-based exploration

6. **P2P Sync Manager**
   neighbor sessions + delta exchange

7. **P2P Task Listener**
   receive announce / claim / result
   update state / task queue

8. Logging / metrics

⚠ Important:
**Task Listener must always run** —
it is the **event source that triggers state transitions**.

---

# BOOT Behavior Tree

* Safety / Geofence / Fault: mostly idle (hover)
* Sync: hello/ack allowed
* Task: ignore incoming tasks

---

# SURVEY Behavior Tree (Primary Operation Mode)

* Task Executor: only execute **VERIFY_TAG tasks already won**
* Exploration Policy: **main active component**
* Sync Manager: normal frequency
* Task Listener: allow verify auctions

---

# VERIFY_TAG Behavior Tree (Task Preemption)

* Task Executor executes full verification sequence:

```
go_to_target
→ descend / approach
→ dwell / scan
→ report result
```

* Exploration: disabled or very low priority
* Sync Manager: still running
* Task Listener: receives tasks but usually does not claim new ones

---

# PATH_VERIFY Behavior Tree

Two possible roles:

---

### A) Path Verifier (won `BECOME_PATH_VERIFIER`)

* Task Executor prioritizes:

```
VERIFY_PATH_SEGMENT
RESCAN_PATH_BUFFER
```

* Exploration disabled or very low
* Sync more aggressive

---

### B) Non-Verifier Drone

* Continue exploration but **less aggressive**
* May still participate in high-priority `VERIFY_TAG` auctions

---

# CONVERGE Behavior Tree

* Task Executor only performs **final critical tasks**:

  * low-confidence mines near path
  * critical segment rescans

* Exploration mostly disabled

* Sync frequency increased

* Task Listener may restrict tasks to high priority

---

# FINISH Behavior Tree

* Stop announcing / claiming tasks
* Explorer enters **hold / hover**
* Sync reduced or stopped
* Publish final logs / summaries

---

# D) Suggested Time Thresholds

Example parameters:

```
T_total = 7:00

T_path_verify_start = 120 s
T_converge_start = 90 s
T_finish = 10 s
```

Meaning:

* Enter **PATH_VERIFY** when **2 minutes remain**
* Enter **CONVERGE** when **90 seconds remain**
* Freeze system **10 seconds before the end**

