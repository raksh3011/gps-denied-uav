# Mission Manager Module

Owner: Person 4. Real `Mission` production from the frozen contract in [docs/INTERFACES.md](INTERFACES.md). `MockMission` still exists and still runs — swapping in `real_mission` is a launch-file change (`real_mission_pipeline.launch.xml` runs it with the rest mocked; `demo_mission.launch.xml` and `full_real_pipeline.launch.xml` run it with default params that exactly match the old golden scenario), matching the workflow in [docs/DEVELOPMENT.md](DEVELOPMENT.md).

## Status

**Unit-tested and contract-tested; last of the four "real" modules to land.** Same tier as World Model/Planning/Safety (not Vehicle's split tier) — everything here runs and is verified with no Gazebo/PX4 needed.

- `uav_mission_core` (C++ library, no ROS dependency): `MissionManager` — a waypoint-sequencing state machine. 7 gtests, all passing.
- `real_mission` (ROS 2 node): loads a mission from ROS parameters (parallel arrays), publishes `/mission/current` (transient_local, matching the contract), subscribes `/localization/state` to advance through waypoints and `/planning/status` for telemetry.
- `tests/contract/test_mission_contracts.py`: subprocess-based, no simulator — includes a genuine multi-leg sequencing test, not just the single-target default.

Not yet done / known rough edge:
- **No mission file format.** The contract's own comment says Mission is "loaded from mission file / operator input" — this implementation uses ROS's own params-file mechanism (see `config/multi_leg_example.yaml`) rather than inventing a bespoke format, which is a deliberate scope choice, not an oversight: a real ground-station/operator UI is a separate, much larger piece of work than sequencing logic.
- **What happens when the mission completes.** Once the last waypoint is reached, `real_mission` publishes a `Mission` with an empty `waypoints` list — and `real_planner_node.cpp` currently treats an empty-waypoints `Mission` as `STATE_FAILED` ("invalid localization or empty mission"), not as some cleaner "mission complete, nothing to do" state. That's an existing behavior in Planning's code, not something changed here — flagged honestly rather than silently patched, since fixing it means touching Person 3's module and should be a deliberate cross-team decision, not a side effect of landing Mission.
- No geofence/altitude-bound enforcement inside Mission itself — `boundary_radius`/`min_altitude`/`max_altitude` are carried through as static fields but nothing in this module checks the vehicle against them. That's arguably a Safety-Supervisor-style redundant check (in the same spirit as Safety's independent obstacle-clearance check, [docs/SAFETY.md](SAFETY.md)), not something bolted on here without being asked.

## Design

```
ROS params (mission_id, waypoint_x/y/z/yaw/acceptance_radius/type[], ...)
        │
        v
  real_mission ──> MissionManager ──> /mission/current (Mission, transient_local)
        ^
        │
/localization/state (position, localization_ok)
```

- **Sequencing lives in Mission, not Planning.** `real_planner_node` only ever reads `mission.waypoints.front()` — it has no concept of a multi-leg route. So advancing through a route has to happen by Mission republishing a shorter `Mission` each time a leg completes, with the next waypoint now at the front. This is exactly what `MissionManager::updatePosition()` does.
- **Arrival uses `Waypoint.acceptance_radius` directly** — the frozen contract already defines that field for exactly this purpose, so no new interface or heuristic was needed. The vehicle's position is compared against the current front waypoint each time `LocalizationState` arrives; within radius, that waypoint is popped.
- **An untrustworthy position never advances the mission.** `updatePosition()` is a no-op whenever `localization_ok` is false, even if the raw position happens to be numerically within the acceptance radius — a position we don't trust isn't evidence of arrival.
- **Static mission fields (`max_speed`, `boundary_radius`, `min_altitude`, `max_altitude`, `mission_id`) are carried through unchanged** across every republish — only the `waypoints` list shrinks.

## Parameters (all on `real_mission`)

| Parameter | Default | Meaning |
|---|---|---|
| `mission_id` | `"golden-scenario-01"` | matches the old MockMission default |
| `max_speed` | 3.0 | m/s |
| `boundary_radius` | 50.0 | meters, geofence radius around waypoint[0] |
| `min_altitude` / `max_altitude` | 1.0 / 20.0 | meters AGL |
| `waypoint_x` / `_y` / `_z` | `[10.0]` / `[0.0]` / `[3.0]` | parallel arrays, one entry per leg |
| `waypoint_yaw` | `[NaN]` | radians, NaN = don't-care |
| `waypoint_acceptance_radius` | `[0.5]` | meters |
| `waypoint_type` | `[Waypoint::TYPE_TARGET]` | per-leg `Waypoint.TYPE_*` |

All six `waypoint_*` arrays must be the same length — a mismatch is logged as an error and `real_mission` publishes an empty mission rather than a silently-wrong one. See `config/multi_leg_example.yaml` for a genuine 3-leg example.

## How to verify

Unit + lint + contract tests, no simulator:

```bash
cd ~/gps-denied-uav && colcon build --packages-select uav_mission && colcon test --packages-select uav_mission && colcon test-result --verbose
```

```bash
cd ~/gps-denied-uav && source install/setup.bash && pytest tests/contract/test_mission_contracts.py -v
```

Isolated swap (rest of the pipeline mocked), default single-target mission:

```bash
ros2 launch uav_bringup real_mission_pipeline.launch.xml
```

A genuine multi-leg mission, standalone:

```bash
ros2 run uav_mission real_mission --ros-args --params-file src/mission/uav_mission/config/multi_leg_example.yaml
```
