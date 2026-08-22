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

Worth being direct about this: the base algorithms are a careful application of two well-established, real-time-proven algorithms — **Lazy Theta*** (Nash, Koenig, Tovey, 2010 — the lazy-evaluation refinement of Nash et al.'s original 2007 Theta*) for the global planner, and **D* Lite** (Koenig & Likhachev, 2002, incremental replanning — the same family of algorithm used on real fielded rovers) for the local planner. Neither is a new algorithm invented here, and claiming otherwise would be dishonest. None of the timing claims below are backed by testing under real flight conditions yet either ("even in extreme conditions" is a claim that needs real test data, not asserted).

What *is* our own contribution is the extension below.

## Algorithmic contribution: confidence-adaptive risk margin (CARM)

The claim, stated precisely so it stays honest:

> **The local planner treats the vehicle's own live localization quality (`LocalizationState.confidence`/`status` from the LIO pipeline) as a first-class, quantized risk signal inside D* Lite's incremental cost structure, re-keying only the cells that carry obstacle-proximity soft cost when the risk band crosses a threshold — so a degrading pose estimate widens the effective obstacle berth in O(|risk cells|) incremental work, without recomputing inflation geometry, without a full re-search, and without breaking D* Lite's admissibility.**

Why this specific mechanism (and its cost profile):

- **Time**: a risk-band crossing touches `updateVertex()` only for cells with nonzero soft cost (`risk_cells_`, maintained incrementally as the map diffs) plus their immediate neighbors, then runs the normal `computeShortestPath()` propagation. Free space — the overwhelming majority of the grid in flight — is untouched. Ticks where the band doesn't cross are a single integer compare, O(1).
- **Space**: one `std::vector<int>` of soft-cost cell ids, one double, one int. No second grid, no per-cell uncertainty field.
- **Stability** ("other costs"): the signal is quantized into 3 bands (NOMINAL / DEGRADED / LOST, with confidence thresholds 0.8 / 0.4 as the fallback when `status` alone doesn't escalate). Raw confidence/covariance is noisy tick-to-tick; applying it continuously would re-key cells every single tick and destroy exactly the bounded-incremental-update property D* Lite exists to provide. Quantization is what makes uncertainty-awareness *compatible with* incrementality.
- **Correctness**: the multiplier only ever scales the *soft* proximity cost (never the base `resolution_` step cost, never occupancy), so edge costs only increase above the Euclidean baseline — the heuristic stays admissible and D* Lite's optimality-w.r.t.-current-costs guarantee is untouched.

The operational rationale: in a GPS-denied stack, the map and the pose come from the *same* LIO estimate. When localization degrades, the vehicle's believed position — and therefore its believed clearance from every obstacle — is less trustworthy, so the rational response is to buy physical margin. CARM makes the planner do that automatically and reversibly (confidence recovers → band drops → paths tighten again), rather than Safety having to halt the vehicle outright.

Honest prior-art positioning (checked 2026-08-20, see sources in the project discussion): uncertainty-aware D* Lite variants exist — notably **URD*/URA*** (Fan et al., 2023), which feeds *semantic terrain-traversability* uncertainty from an image segmentation network into D* Lite for off-road ground vehicles — and *perception-aware* planners exist that shape paths to keep localization healthy (Costante et al., 2016; and localization-uncertainty-corridor planners for urban UAVs). Uncertainty-aware LIO also exists (**UA-LIO**, 2025) but stops at the odometry output. We did not find published work that closes this specific loop — live LIO ego-pose confidence quantized into banded soft-cost multipliers inside an incremental D* Lite for an aerial vehicle, with the incremental re-key restricted to a maintained soft-cost cell set. That's a narrow but genuine gap, and it's the honest scope of the claim: **a novel integration mechanism with a specific complexity guarantee — not a new search algorithm.** Before presenting this externally (paper, patent, pitch), a proper literature review beyond a web search is still required.

Implementation: `DStarLitePlanner::setLocalizationRisk()` in [dstar_lite_planner.cpp](../src/planning/uav_planning/src/dstar_lite_planner.cpp), wired per-tick in `real_planner_node.cpp`; behavior pinned by the three `RiskBand*`/`DegradedLocalization*` tests in `test_dstar_lite_planner.cpp`.

What's specific to this project beyond CARM is the combination and the integration: Theta* seeding a persistent D* Lite instance that then runs incrementally every tick against a live occupancy grid built from `LocalMap` + inflated `ObstacleSet`, wired into this exact interface contract — a legitimate, defensible engineering choice for "shortest global path" + "low-latency local reaction", but engineering, not algorithm invention.

## Real-time hardening: deadline-bounded incremental replanning

`DStarLitePlanner::update()`/`setLocalizationRisk()` take an optional wall-clock deadline (`Clock::time_point`, default = none). **This is explicitly not presented as a novel algorithm.** Checked prior art (2026-08-21): **Anytime Dynamic A\*** (Likhachev, Ferguson, Gordon, Stentz, Thrun, 2005) already combines bounded-time anytime search with incremental replanning — adding "a deadline" to D* Lite on its own would just be re-deriving that territory, and claiming it as ours would repeat the exact mistake CARM's own prior-art check was built to avoid.

What's actually here is narrower: D* Lite's own textbook invariant is that its `g_`/`rhs_` state and priority queue are valid to interrupt and resume at any point — that is the entire premise of an incremental planner. `computeShortestPath()` already had an expansion-count cap (`max_compute_iterations_`); it now *also* checks a wall-clock deadline every iteration, so a single tick can never exceed the deadline regardless of obstacle density, only the two established bounds it already had (expansion count, or the deadline) trigger first. `real_planner_node` computes that deadline as `kDeadlineFraction` (0.7) of the tick period, leaving headroom for `inflateObstacles`/trajectory generation/publishing in the same tick. A tick that hits the deadline reports it via `lastComputeHitDeadline()` (currently `RCLCPP_DEBUG`-logged, not surfaced in `PlannerStatus` — see Next tasks) and returns the best-known path so far: still guaranteed collision-free (`edgeCost()` refuses occupied cells regardless of convergence state), possibly not yet optimal, corrected on the next tick as the persistent search continues from where it left off.

Regression tests: `AlreadyPastDeadlineStopsImmediatelyAndSafely`, `GenerousDeadlineConvergesNormallyAndReportsSo`, `InterruptedSearchResumesAndConvergesAcrossCalls` (the last one uses a tiny `max_compute_iterations_` rather than real timing to test the same "safe to interrupt and resume" property deterministically, without wall-clock flakiness in CI).

## Benchmarked insights

Real, measured numbers — not asserted — from `bench_planning`, a standalone tool (no pass/fail, not part of `colcon test`/CI) at the exact arena scale (120x60x40 @ 0.25m, 8 obstacles) that surfaced both the stale-queue and eager-Theta* bugs:

```bash
colcon build --packages-select uav_planning
./install/uav_planning/lib/uav_planning/bench_planning
```

It reports five things, each chosen to be the honest comparison rather than a flattering one: (1) cold from-scratch global search cost, Lazy Theta* vs. plain A*; (2) D* Lite's steady-state incremental `update()` cost vs. what re-running Theta* from scratch every tick would cost instead — the actual measured payoff of being incremental, on this machine, not a complexity argument; (3) CARM's risk-band-switch cost vs. a full re-`initialize()` — same idea, for CARM specifically; (4) the deadline hardening's real worst-case single-call latency under an artificially tiny budget, confirming empirically that a tick can't run away; (5) `DStarLitePlanner`'s approximate memory footprint at this grid scale.

*Numbers pending a run on real hardware — this section gets the actual output appended once that happens, not filled in ahead of time.*

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

**Fixed (part 1 — stale queue entries):** `AStarPlanner`/`ThetaStarPlanner`'s open-list had no stale-entry check — when a cell's cost was revised, the old queue entry for it was never invalidated, so popping it later re-ran a full 6-neighbor expansion for no benefit. Fixed with the same lazy-deletion pattern `DStarLitePlanner` already used (store `g` at push time, skip a popped entry if it no longer matches the current best `g_score`).

**Fixed (part 2 — the actual cost of the problem):** part 1 alone wasn't enough. The regression test (`ThetaStarPlanner, FindsPathOnLargeOpenGridWithSparseObstacles`, at the visual demo's real 30x15x10m/0.25m arena scale) still failed after part 1 — 150 real seconds to exhaust `max_expansions` without ever reaching a goal that was trivially reachable. The deeper issue: eager Theta* calls `traceLine` (an O(distance) raycast) for **every one of a node's 6 neighbors, at every expansion** — but most pushed candidates are superseded by a better path before they're ever actually expanded, so nearly all of that raycasting is wasted work on paths the search never uses. Switched `ThetaStarPlanner` to **Lazy Theta*** (Nash, Koenig, Tovey, 2010 — a published, standard refinement, not something invented here): the line-of-sight check now happens once per node actually popped/expanded, against that node's own grandparent, cutting raycasts from O(6 x expansions) to O(expansions). Same regression test now passes in well under a second. This also means the earlier "Theta*" naming throughout this doc and the code comments now specifically means Lazy Theta* — updated above.
2. Resolve the rolling-map caveat above once World Model's real re-centering behavior exists — likely needs `DStarLitePlanner` to support shifting its snapshot when the origin moves by a known offset, rather than a hard re-initialize every time.
3. Acceleration-limited/smoothed trajectory generation (e.g. trapezoidal velocity profile or a spline through the thinned waypoints) instead of instantaneous velocity changes at each waypoint.
4. Tune `kHardMarginM`/`kSoftMarginM`/`kSoftCostWeight` (currently reasonable-guess constants in `real_planner_node.cpp`) against real vehicle dimensions and desired obstacle standoff.
5. Run the golden scenario (`docs/TESTING.md`) with real Planning once real Localization also has confirmed odometry (see [docs/LOCALIZATION.md](LOCALIZATION.md)), and compare against the mocked baseline.
6. Run `bench_planning` on real hardware and paste its actual output into the "Benchmarked insights" section above — it's a placeholder until then, deliberately not filled in with estimates.
7. Surface `DStarLitePlanner::lastComputeHitDeadline()` in `PlannerStatus` (a new field, or reuse `message`) so a tick that returned a stale/unconverged path is visible to Safety/Mission, not just `RCLCPP_DEBUG`-logged.
