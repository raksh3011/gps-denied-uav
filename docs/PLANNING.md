# Planning Module

Owner: Person 3. Real global + local planning, boundary checking, and trajectory generation, layered on the frozen `Trajectory`/`PlannerStatus` contract in [docs/INTERFACES.md](INTERFACES.md). `MockPlanner` still exists and still runs — swapping in `real_planner` is a launch-file change, not an interface change (`mock_pipeline.launch.xml` -> `real_planning_pipeline.launch.xml`), matching the workflow in [docs/DEVELOPMENT.md](DEVELOPMENT.md).

## Status

**Unit-tested and buildable; not yet run against real Gazebo sensor data.** Deliberately scoped this way: Planning only consumes `Mission`/`LocalizationState`/`LocalMap`/`ObstacleSet` — all synthesizable directly in tests — so the whole algorithm core was built and verified without touching Gazebo/PX4 at all. (Localization's simulation integration turned into a very long debugging session for reasons that had nothing to do with the algorithm itself; Planning's core logic is proven correct independent of that entirely.)

- `uav_planning_core` (C++ library, no ROS dependency): `Grid3D`, `AStarPlanner`, `ThetaStarPlanner`, `DStarLitePlanner`, `BoundaryChecker`, `TrajectoryGenerator` — each has its own `gtest` unit tests (`colcon test --packages-select uav_planning`), no simulator or even `rclcpp::init` required.
- `real_planner` (ROS 2 node): wires the above into the same topics/QoS `MockPlanner` uses.
- `tests/contract/test_planning_contracts.py`: drives `real_planner` as a real subprocess over real ROS topics with synthetic `Mission`/`LocalizationState`/`LocalMap`/`ObstacleSet` — still no Gazebo/PX4 needed.

Not yet done:
- Never run with real (non-synthetic) `LocalMap`/`ObstacleSet` from a real `MockWorldModel` deployment at scale, or against real Localization's actual pose noise/covariance — none of the timings below are measured on realistic map sizes yet.
- No acceleration limiting/smoothing in `TrajectoryGenerator` — constant-speed straight segments between waypoints, which is kinematically abrupt at direction changes. Fine for proving the contract; not flight-quality.
- `DStarLitePlanner` assumes the local map's origin stays fixed between ticks. A genuinely *rolling* (re-centering) local map forces a full re-`initialize()`, losing the incremental benefit for that one tick — see below.

## On "novel IP"

Worth being direct about this: what's built here is a careful application of two well-established, real-time-proven algorithms — **Theta*** (Nash et al., 2007, any-angle path planning) for the global planner, and **D* Lite** (Koenig & Likhachev, 2002, incremental replanning — the same family of algorithm used on real fielded rovers) for the local planner. Neither is a new algorithm invented here. What's specific to this project is the combination and the integration: Theta* seeding a persistent D* Lite instance that then runs incrementally every tick against a live occupancy grid built from `LocalMap` + inflated `ObstacleSet`, wired into this exact interface contract. That's a legitimate, defensible engineering choice for "shortest global path" + "low-latency local reaction" — but claiming the underlying algorithms themselves as novel would be dishonest, and none of the numbers below are backed by testing under real flight conditions yet ("even in extreme conditions" is a claim that needs real test data, not asserted).

## Architecture

```
Mission ──┐
LocalizationState ─┼─> real_planner ──> Trajectory (map frame, time-parameterized)
LocalMap ──┤              │            └─> PlannerStatus
ObstacleSet ┘              │
                           v
                       Grid3D  (occupancy + inflated obstacles + soft cost)
                        │   │
          on goal change│   │every tick
                        v   v
              ThetaStarPlanner   DStarLitePlanner (persistent across ticks)
              (global reference,        │
               not published)           v
                                  TrajectoryGenerator ──> published Trajectory
                 ^
            BoundaryChecker (clamps goal to geofence + altitude)
```

Each tick (`rate_hz` param, default 10 Hz, matching `MockPlanner`):

1. Gate on having a valid `Mission` + `LocalizationState` + `LocalMap` — `STATE_IDLE` if any are missing, `STATE_FAILED` if localization is unhealthy or the mission has no waypoints (identical gating to `MockPlanner`, so downstream Safety/Mission see the same state machine either way).
2. Build a `Grid3D` from `LocalMap`'s own occupancy array (`0` = free, per the LocalMap contract, anything else = occupied), then `inflateObstacles()` from the latest `ObstacleSet`: cells within `radius + hard_margin` of an obstacle are hard-occupied (never routed through), cells further out within `soft_margin` carry a distance-decaying added cost (a real cost function — see below — not just occupied/free).
3. `BoundaryChecker::clamp()` the requested goal (`Mission.waypoints[0]`) against the geofence (`boundary_radius` around that same waypoint, per the `Mission` contract) and altitude limits, so the planner is never asked to reach an illegal point.
4. **If the (clamped) goal changed since last tick** (or this is the first valid tick): run `ThetaStarPlanner::plan()` once — any-angle search, so the reference path length is close to the true Euclidean-shortest path rather than a 6-connected grid's axis-aligned staircase — purely to sanity-check reachability and compute a total-path-length figure for the progress metric, then `DStarLitePlanner::initialize()` fresh against the same start/goal/grid.
5. **Every tick**, regardless of whether the goal changed: `DStarLitePlanner::update()` — diffs the current `Grid3D` against its own internally held snapshot from last tick, propagates updates only from cells that actually changed (moved obstacle, vehicle motion), and returns the current best path. This is what actually gets published — it's the one with a bounded-latency incremental-update guarantee, which is the actual "no delay" property a local planner needs; Theta* alone re-searches from scratch and doesn't have that property.
6. `TrajectoryGenerator::generate()` — thins the raw path down to `~2.5x` map resolution spacing, then time-parameterizes at `Mission.max_speed` with zero velocity at the final point (arrival).

## Why D* Lite instead of "full replan every tick"

An earlier version of this module just re-ran A* from scratch every tick. That's correct but has no latency bound — worst case, every single tick pays for a full search over the entire grid, regardless of how much of the environment actually changed. D* Lite holds `g`/`rhs` state across ticks and a priority queue seeded from the goal; when the grid changes, only the affected cells (and whatever propagates from them) get re-examined. In the common case — a static or slowly-changing environment with the vehicle moving toward a stationary goal — this is dramatically cheaper per tick than a full re-search, which is the actual mechanism behind "no delay, no drift": it's not a claim about compute magic, it's about not repeating work that didn't need repeating.

The known cost of this: `DStarLitePlanner` identifies map cells by integer grid index, and its internal snapshot assumes those indices mean the same world location tick to tick. If `LocalMap`'s origin genuinely re-centers on the vehicle (a literal "rolling" map, which `docs/ARCHITECTURE.md`'s description of World Model's "local rolling map" suggests is the eventual design), every re-centering event invalidates that assumption. `real_planner_node.cpp` detects this (origin/size/resolution mismatch) and forces a full `initialize()` rather than silently using stale/wrong data — correct, but it means the incremental benefit is lost on exactly the ticks where the map just moved. This needs revisiting once World Model's real re-centering behavior and its actual update frequency are known.

## Confidence / progress semantics

`PlannerStatus.progress` is `1 - (remaining distance to goal) / (Theta* global path length computed when the goal was last set)`. Unlike the earlier every-tick-replan version, the denominator is now stable across ticks (fixed at goal-change time), so progress increases monotonically as the vehicle approaches the goal along a *stable* plan — it only resets when the goal itself changes, not every time D* Lite's local path shape shifts slightly.

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

1. Run against a real `MockWorldModel`-scale `LocalMap` (larger grid, denser obstacle sets) and actually measure tick time for both `ThetaStarPlanner` (on goal change) and `DStarLitePlanner::update()` (every tick) — nothing here has been profiled against realistic map sizes, and `max_expansions`/`max_compute_iterations` are unvalidated guesses.
2. Resolve the rolling-map caveat above once World Model's real re-centering behavior exists — likely needs `DStarLitePlanner` to support shifting its snapshot when the origin moves by a known offset, rather than a hard re-initialize every time.
3. Acceleration-limited/smoothed trajectory generation (e.g. trapezoidal velocity profile or a spline through the thinned waypoints) instead of instantaneous velocity changes at each waypoint.
4. Tune `kHardMarginM`/`kSoftMarginM`/`kSoftCostWeight` (currently reasonable-guess constants in `real_planner_node.cpp`) against real vehicle dimensions and desired obstacle standoff.
5. Run the golden scenario (`docs/TESTING.md`) with real Planning once real Localization also has confirmed odometry (see [docs/LOCALIZATION.md](LOCALIZATION.md)), and compare against the mocked baseline.
