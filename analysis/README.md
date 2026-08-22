# Independent Path-Planning Analysis

A standalone, non-functional demonstration comparing our production planning stack (Lazy Theta* + D* Lite + Margasoochi, the confidence-adaptive risk margin) against a traditional grid-search baseline (plain 6-connected A*, replanned from scratch on every environment change).

## This is not part of the flying system

`compare_planners` is a read-only measurement tool. It **calls** the same `uav_planning_core` library the real vehicle uses, exactly the way a unit test does, but:

- It is not launched by any launch file, ROS node, or mock.
- It is not part of `colcon test` / CI — nothing here gates a build or a merge.
- It writes zero data back into the library. Every measurement is taken externally (wall-clock time around an existing public call, or geometry computed from the path/obstacle data the library already returns) — no planner file was touched or instrumented to produce these numbers.
- Deleting the entire `analysis/` directory changes nothing about how the drone flies.

```
Path Planner (uav_planning_core, unmodified)
        │
        v
  Test Scenarios (analysis/compare_planners.cpp)
        │
        v
  Recorded Results (analysis/results/results.json — real measured numbers)
        │
        v
  Independent Analysis (this README + the numbers)
        │
        v
  Visualization (the published report — see below)
```

## Honest baseline definition

"Traditional" here means **textbook grid-based A\*, replanned from scratch every time the environment changes** — a real, widely-cited standard in mobile-robot path planning, not a strawman weakened to make ours look better. It's our own `AStarPlanner`: unmodified, 6-connected, admissible-heuristic A*, with no line-of-sight shortcuts and no incremental reuse between calls.

"Ours" is reported as **two separate things, deliberately not flattened together**:

- **Global reference (Lazy Theta\*)** — what the real system computes once per goal change. Genuinely smoother and shorter than A*'s output; that's the whole point of any-angle search.
- **Flown local path (D\* Lite + Margasoochi)** — what the real system actually publishes as `/planning/trajectory` every tick. This is a 6-connected grid search internally, same as A* — **it does not inherit Theta*'s any-angle shortcuts on its own.** Reporting only the Theta* number here would overstate what's actually flown; reporting only the D* Lite number would hide the real value of the any-angle global reference. Both are shown so nothing is cherry-picked.

Where "traditional" has no equivalent mechanism at all (Margasoochi's confidence-adaptive margin — plain A*/D* Lite have no concept of localization confidence), the result is marked **N/A — no adaptive mechanism**, not `0`, since `0` would misleadingly imply "present but ineffective."

## Scenarios (identical inputs to both approaches)

1. **Open field** — no obstacles; measures pure search overhead.
2. **Sparse obstacles** — the same 8-pillar arena used throughout this project's own tests and demos (`demo_mission.launch.xml`, the `ThetaStarPlanner` regression test) — not a new, cherry-picked layout.
3. **Dense/cluttered obstacles** — a harder, more constrained field.
4. **Dynamic obstacle appears mid-flight** — measures re-plan latency and whether the rerouted path stays clear.
5. **Localization degradation event** — confidence drops mid-flight; measures Margasoochi's margin response (traditional: N/A, see above).

## Running it

```bash
cd ~/gps-denied-uav
colcon build --packages-select uav_planning   # builds compare_planners alongside bench_planning
./install/uav_planning/lib/uav_planning/compare_planners
```

Writes `analysis/results/results.json` (or wherever you run it from — pass an output path as the first argument to control that) with every measured number, plus a console summary table.

## Status

**Not yet run.** No numbers in this document or any published report are real until `compare_planners` has actually executed on real hardware and its `results.json` has been used to build the visualization. Anything shown before that point is explicitly labeled illustrative/simulated, per the same standard the rest of this project holds itself to (see `docs/PLANNING.md`'s own "Benchmarked insights" section for the precedent).
