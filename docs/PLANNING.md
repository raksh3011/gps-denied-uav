# Planning Module

Owner: Person 3. Real global planning + boundary checking + trajectory generation, layered on the frozen `Trajectory`/`PlannerStatus` contract in [docs/INTERFACES.md](INTERFACES.md). `MockPlanner` still exists and still runs — swapping in `real_planner` is a launch-file change, not an interface change (`mock_pipeline.launch.xml` -> `real_planning_pipeline.launch.xml`), matching the workflow in [docs/DEVELOPMENT.md](DEVELOPMENT.md).

## Status

**Unit-tested and buildable; not yet run against real Gazebo sensor data.** Deliberately scoped this way: Planning only consumes `Mission`/`LocalizationState`/`LocalMap`/`ObstacleSet` — all synthesizable directly in tests — so the whole algorithm core was built and verified without touching Gazebo/PX4 at all. (Localization's simulation integration turned into a very long debugging session for reasons that had nothing to do with the algorithm itself; Planning's core logic is proven correct independent of that entirely.)

- `uav_planning_core` (C++ library, no ROS dependency): `Grid3D`, `AStarPlanner`, `BoundaryChecker`, `TrajectoryGenerator` — each has its own `gtest` unit tests (`colcon test --packages-select uav_planning`), no simulator or even `rclcpp::init` required.
- `real_planner` (ROS 2 node): wires the above into the same topics/QoS `MockPlanner` uses.
- `tests/contract/test_planning_contracts.py`: drives `real_planner` as a real subprocess over real ROS topics with synthetic `Mission`/`LocalizationState`/`LocalMap`/`ObstacleSet` — still no Gazebo/PX4 needed.

Not yet done:
- Never run with real (non-synthetic) `LocalMap`/`ObstacleSet` from a real `MockWorldModel` deployment at scale, or against real Localization's actual pose noise/covariance.
- No acceleration limiting/smoothing in `TrajectoryGenerator` — constant-speed straight segments between waypoints, which is kinematically abrupt at direction changes. Fine for proving the contract; not flight-quality.
- Replanning strategy is "recompute the whole global path every tick" (see below) — works, but isn't a real local planner in the sense of reacting to a *moving* obstacle between two planning ticks.

## Architecture

```
Mission ──┐
LocalizationState ─┼─> real_planner ──> Trajectory (map frame, time-parameterized)
LocalMap ──┤              │            └─> PlannerStatus
ObstacleSet ┘              │
                           v
              Grid3D <- AStarPlanner -> TrajectoryGenerator
                 ^
            BoundaryChecker (clamps goal to geofence + altitude)
```

Each tick (`rate_hz` param, default 10 Hz, matching `MockPlanner`):

1. Gate on having a valid `Mission` + `LocalizationState` + `LocalMap` — `STATE_IDLE` if any are missing, `STATE_FAILED` if localization is unhealthy or the mission has no waypoints (identical gating to `MockPlanner`, so downstream Safety/Mission see the same state machine either way).
2. Build a `Grid3D` from `LocalMap`'s own occupancy array (`0` = free, per the LocalMap contract, anything else = occupied), then `inflateObstacles()` from the latest `ObstacleSet`: cells within `radius + hard_margin` of an obstacle are hard-occupied (never routed through), cells further out within `soft_margin` carry a distance-decaying added cost (a real cost function — see below — not just occupied/free).
3. `BoundaryChecker::clamp()` the requested goal (`Mission.waypoints[0]`) against the geofence (`boundary_radius` around that same waypoint, per the `Mission` contract) and altitude limits, so the planner is never asked to reach an illegal point.
4. `AStarPlanner::plan()` — 6-connected 3D grid search, edge cost = grid step distance + `Grid3D`'s soft obstacle-proximity cost. Bounded by `max_expansions` so a worst-case maze-like map can't blow a planning tick's time budget; returns an empty path (→ `STATE_FAILED`) rather than search forever.
5. `TrajectoryGenerator::generate()` — thins the raw one-point-per-grid-cell path down to `~2.5x` map resolution spacing, then time-parameterizes at `Mission.max_speed` with zero velocity at the final point (arrival).

## Why "replan from scratch every tick" instead of a real local planner

Person 3's ownership list separates a global planner from a local planner with its own dynamic replanning. What's built is only the global half, run repeatedly: every tick, `real_planner` throws away whatever it planned last time and re-runs full A* against the *current* `LocalMap`/`ObstacleSet`. This does react to obstacles that appear/move between ticks — the next tick's grid reflects them — but it's not what "local planner" usually means (a fast, short-horizon planner that adjusts the existing trajectory locally, falling back to the global planner only when the local adjustment isn't enough). That's real, useful future work — see Next tasks.

## Confidence / progress semantics

`PlannerStatus.progress` is `1 - (remaining distance to goal) / (total distance from the path's start to goal)`, recomputed fresh each tick from the current position — not tied to a persistent plan, since there isn't one (see above). It's a reasonable "how far along" signal but will jump if the vehicle deviates significantly and the planner reroutes.

## Build and test

Everything here runs without Gazebo, PX4, or even the mocked pipeline — just `colcon build` + `pytest`:

```bash
colcon build --symlink-install --packages-select uav_planning
colcon test --packages-select uav_planning && colcon test-result --verbose   # gtest unit tests
pytest tests/contract/test_planning_contracts.py -v                          # node contract tests
```

To see it running against the rest of the mocked pipeline:

```bash
source install/setup.bash
ros2 launch uav_bringup real_planning_pipeline.launch.xml
# in another terminal:
ros2 topic echo /planning/trajectory
ros2 topic echo /planning/status
```

## Next tasks, roughly in order

1. Run against a real `MockWorldModel`-scale `LocalMap` (larger grid, denser obstacle sets) and profile `AStarPlanner`'s tick time — `max_expansions` and the grid resolution/size tradeoff haven't been tuned against realistic map sizes yet.
2. Acceleration-limited/smoothed trajectory generation (e.g. trapezoidal velocity profile or a spline through the thinned waypoints) instead of instantaneous velocity changes at each waypoint.
3. A real local planner: given the existing global path, do a fast local check/adjustment against the latest `ObstacleSet` first, and only fall back to a full A* replan when the local adjustment can't clear the obstacle — replacing "replan from scratch every tick."
4. Tune `kHardMarginM`/`kSoftMarginM`/`kSoftCostWeight` (currently reasonable-guess constants in `real_planner_node.cpp`) against real vehicle dimensions and desired obstacle standoff.
5. Run the golden scenario (`docs/TESTING.md`) with real Planning once real Localization also has confirmed odometry (see [docs/LOCALIZATION.md](LOCALIZATION.md)), and compare against the mocked baseline.
