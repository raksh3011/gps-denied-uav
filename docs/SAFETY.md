# Safety Supervisor Module

Owner: Person 4. Real `VehicleCommand`/`SystemHealth` production from the frozen contracts in [docs/INTERFACES.md](INTERFACES.md). `MockSafety` still exists and still runs — swapping in `real_safety` is a launch-file change (`real_safety_pipeline.launch.xml` runs it with the rest mocked; `demo_mission.launch.xml` and `full_real_pipeline.launch.xml` run it as part of the full real stack), matching the workflow in [docs/DEVELOPMENT.md](DEVELOPMENT.md).

## Status

**Unit-tested and buildable; not yet run against real hardware faults.** Same scoping as World Model and Planning: the ROS-free core (`SafetyMonitor`) was built and verified entirely with synthetic inputs — no Gazebo/PX4 required — and the contract tests drive `real_safety` as a subprocess over real topics.

- `uav_safety_core` (C++ library, no ROS dependency): `SafetyMonitor` — the state machine below. gtest-covered (13 tests: healthy/degraded/stale/invalid combinations, the obstacle-clearance override, sustained-loss escalation and its recovery reset).
- `real_safety` (ROS 2 node): subscribes every topic the frozen contract lists Safety as a consumer of — `LocalizationState`, `LocalMap`, `ObstacleSet`, `Trajectory`, `PlannerStatus` — publishes `/safety/vehicle_command` + `/safety/system_health` at `rate_hz` (default 20), same topics/QoS as `MockSafety`.
- `tests/contract/test_safety_contracts.py`: subprocess-based, synthetic inputs, no simulator.

Not yet done:
- Never run against real hardware/sim faults — a real PX4 disconnect, a real LiDAR dropout, a real GPS-denied drift event. Parameter defaults (`staleness_timeout_s`, `lost_hold_timeout_s`, `min_obstacle_clearance_m`) are reasonable-guess constants, same caveat as Planning's margin constants.
- `vehicle_link_level` (one of `SystemHealth`'s four per-subsystem fields) has no producer yet — the frozen contract doesn't give Safety a feedback topic from the Vehicle/PX4 Interface, so it's hardcoded `LEVEL_OK`. This becomes real once that module exists and needs its own feedback path (a contract change, not something Safety can fix alone).
- No smoothing/rate-limiting on the command handoff — a HOLD-to-POSITION-to-HOLD flap (e.g. localization bouncing right at the staleness boundary) publishes every transition as-is. Not yet a problem since `valid=false` already means "reject and hold," but worth watching once real PX4 offboard-mode timing is in the loop.

## Design

```
LocalizationState ─┐
Trajectory ─────────┤
LocalMap ───────────┼─> real_safety (SafetyMonitor) ─> VehicleCommand (/safety/vehicle_command)
ObstacleSet ────────┤                                └─> SystemHealth (/safety/system_health)
PlannerStatus ──────┘
```

The core design decision, and the reason Safety and Planning's Margasoochi ([docs/PLANNING.md](PLANNING.md)) are meant to be read together:

- **DEGRADED localization does not block the command.** Planning's own Margasoochi (confidence-adaptive risk margin) already widens obstacle standoff and reacts to a degrading `LocalizationState.confidence`/`status` — that's Margasoochi's whole job. Safety deferring to it (forwarding the command, only downgrading reported health to `LEVEL_WARN`) avoids two modules independently and redundantly reacting to the same signal in possibly-conflicting ways. Safety only actively intervenes when localization is fully **LOST**, or when data is missing/stale/invalid — situations Margasoochi cannot compensate for because there's nothing trustworthy left to plan against.
- **A momentary fault gets `valid=false`, not an active mode change.** Per the `VehicleCommand` contract, `valid=false` means the consumer rejects the command and holds the last safe one — already a safe default. Spamming `MODE_HOLD` for a single stale sample would just be noise.
- **A SUSTAINED loss of localization escalates to an explicit `MODE_LAND` (`valid=true`).** Holding forever on a position estimate that's confirmed gone isn't actually safe — landing in place is. `MODE_RTL` is deliberately not used here: RTL needs a trustworthy position to navigate home, which is exactly what's missing. The sustained-loss timer (`lost_hold_timeout_s`, default 3s) resets the moment localization recovers — a brief blip doesn't count toward it.
- **An independent obstacle-clearance check is Safety's own defense in depth.** It does not trust that Planning's `valid=true` is bug-free: it checks the trajectory's next few points directly against the live `ObstacleSet` (`min_obstacle_clearance_m`, default 0.3m) and overrides with `HOLD` if any of them sit inside an obstacle, regardless of what Planning claims. This is deliberately a second, independently-computed check, not a call into Planning's own logic — the whole point is to catch a bug or a race condition in the module upstream of Safety, not to trust it.

All five per-subsystem faults funnel into `SystemHealth.active_faults` as short codes: `LOC_STALE`, `LOC_NOT_OK`, `LOC_LOST`, `LOC_DEGRADED`, `TRAJ_STALE`, `TRAJ_INVALID`, `MAP_INVALID`, `MAP_STALE`, `PLANNER_FAILED`, `OBSTACLE_CLEARANCE_VIOLATION`, `SUSTAINED_LOSS_LANDING`.

## Parameters (all on `real_safety`)

| Parameter | Default | Meaning |
|---|---|---|
| `staleness_timeout_s` | 0.5 | max age for LocalizationState/Trajectory before treated as absent |
| `map_staleness_timeout_s` | 1.0 | matches LocalMap contract's own ">1s = stale" guidance |
| `lost_hold_timeout_s` | 3.0 | sustained localization loss before HOLD escalates to LAND |
| `min_obstacle_clearance_m` | 0.3 | independent redundant clearance check margin |
| `rate_hz` | 20.0 | publish rate (VehicleCommand contract: >=2Hz required, target 20-50Hz) |

## How to verify

Unit + lint + contract tests, no simulator:

```bash
cd ~/gps-denied-uav && colcon build --packages-select uav_safety && colcon test --packages-select uav_safety && colcon test-result --verbose
```

```bash
cd ~/gps-denied-uav && source install/setup.bash && pytest tests/contract/test_safety_contracts.py -v
```

Isolated swap (rest of the pipeline mocked), or as part of the full real stack:

```bash
ros2 launch uav_bringup real_safety_pipeline.launch.xml
```
