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

**Status:** real world model implemented and unit/contract-tested — `real_world_model` backed by a ROS-free core (`VoxelMapper` rolling hit-count occupancy window, `clusterOccupied`, `ObstacleTracker` with velocity/class estimation). Not yet run against real FAST-LIO2 cloud output; parameter tuning against real data pending. See [docs/WORLD_MODEL.md](WORLD_MODEL.md).

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

**Status:** real global planner (Theta*, any-angle) and real local planner (D* Lite incremental replanning with the confidence-adaptive risk margin — see the "Algorithmic contribution" section of [docs/PLANNING.md](PLANNING.md)), plus boundary checking and trajectory generation, all implemented and unit/contract-tested — no Gazebo/PX4 needed for any of it.

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

**Status:** real Safety Supervisor implemented and unit/contract-tested — `real_safety` backed by a ROS-free core (`SafetyMonitor`: staleness/validity gating across all five subscribed contracts, an independent obstacle-clearance check that doesn't trust Planning's own `valid` flag, sustained-localization-loss escalation to an explicit `MODE_LAND`). Deliberately defers to Planning's CARM for DEGRADED localization rather than double-reacting — see [docs/SAFETY.md](SAFETY.md).

Real Vehicle/PX4 Interface's core (`Px4CommandBridge`: ENU->NED conversion, one-shot vs. streamed commands, an arm/offboard warm-up state machine) is implemented and unit-tested (15 gtests, no ROS/PX4 dependency) — but unlike every other "real" module in this repo, the actual PX4 wiring (`real_vehicle_node`) has never been compiled or run, since it needs `px4_msgs` vendored (`uav_vehicle.repos`) and a live PX4 instance neither exists on the machine this was built on. **Read [docs/VEHICLE.md](VEHICLE.md)'s Status section before assuming any of the PX4-facing part works** — three specific, named risk areas (topic names, `VehicleStatus` enum values, px4_msgs branch/PX4 firmware version match) are called out there. Mission is still a mock.

## Cross-cutting

Interface definitions (`src/interfaces/uav_interfaces`) are jointly owned — no single person merges a change without the other three reviewing, per [GIT_WORKFLOW.md](GIT_WORKFLOW.md#rules). Conventions (`docs/CONVENTIONS.md`) are likewise frozen by team agreement, not by any one owner.
