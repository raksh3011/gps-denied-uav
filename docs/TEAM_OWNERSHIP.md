# Team Ownership

## Person 1 — Localization

**Feature branch:** `feature/localization`
**Packages:** `src/localization/uav_localization`

**Owns:**
- LiDAR interface, IMU interface
- Timestamp synchronization
- LiDAR-Inertial Odometry (LIO) integration
- Pose/velocity/orientation estimation, covariance
- Localization confidence and health reporting

**Provides:** `LocalizationState` (`/localization/state`)
**Consumes:** raw LiDAR/IMU sensor data (via `MockLiDAR`/`MockIMU` until sensors/sim are wired up)

**Can develop independently using:** `MockLiDAR`, `MockIMU`

**Status:** real localization scaffolding in progress — `lio_state_bridge` adapts a vendored FAST-LIO2 backend to the `LocalizationState` contract. `MockLocalization` keeps running unaffected; the rest of the team is not blocked on this. See [docs/LOCALIZATION.md](LOCALIZATION.md) for what's done, what's a placeholder, and the next tasks.

## Person 2 — World Model / Perception

**Feature branch:** `feature/world-model`
**Packages:** `src/world_model/uav_world_model`

**Owns:**
- LiDAR preprocessing, filtering/downsampling
- Local rolling map, voxel/occupancy representation
- Obstacle representation and map updates
- Environment queries

**Provides:** `LocalMap` (`/world_model/local_map`), `ObstacleSet` (`/world_model/obstacles`)
**Consumes:** `LocalizationState`

**Can develop independently using:** `MockLocalization`

## Person 3 — Planning

**Feature branch:** `feature/planning`
**Packages:** `src/planning/uav_planning`

**Owns:**
- Global planner, planning cost function, terrain/map constraints
- Local planner, dynamic replanning, trajectory generation
- Boundary checking

**Provides:** `Trajectory` (`/planning/trajectory`), `PlannerStatus` (`/planning/status`)
**Consumes:** `Mission`, `LocalizationState`, `LocalMap`, `ObstacleSet`

**Can develop independently using:** `MockLocalization`, `MockWorldModel`, `MockMission`

**Status:** real global planner (A* over an inflated occupancy grid), boundary checking, and trajectory generation are implemented and unit/contract-tested — no Gazebo/PX4 needed for any of it. A real *local* planner (fast local replanning against moving obstacles, rather than a full global replan every tick) is not yet built. See [docs/PLANNING.md](PLANNING.md).

## Person 4 — Mission / Safety / Integration

**Feature branch:** `feature/mission-safety`
**Packages:** `src/mission/uav_mission`, `src/safety/uav_safety`, `src/vehicle/uav_vehicle`, `src/simulation/uav_bringup`

**Owns:**
- Mission Manager, mission state machine, mission format
- Safety Supervisor, watchdogs, failure/recovery states
- PX4 interface, Gazebo/PX4 integration
- System-level launch/configuration, integration testing, CI

**Provides:** `Mission` (`/mission/current`), `SystemHealth` (`/safety/system_health`), `VehicleCommand` (`/safety/vehicle_command`)
**Consumes:** `PlannerStatus`, `Trajectory`, `LocalizationState`

**Can develop independently using:** `MockPlanner`, `MockLocalization`

## Cross-cutting

Interface definitions (`src/interfaces/uav_interfaces`) are jointly owned — no single person merges a change without the other three reviewing, per [GIT_WORKFLOW.md](GIT_WORKFLOW.md#rules). Conventions (`docs/CONVENTIONS.md`) are likewise frozen by team agreement, not by any one owner.
