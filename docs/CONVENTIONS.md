# Engineering Conventions

These are frozen for the initialization phase. Do not deviate without team agreement — interface changes require team approval per [GIT_WORKFLOW.md](GIT_WORKFLOW.md).

## Units

SI units everywhere: meters, meters/second, meters/second², radians, radians/second, seconds, kilograms. Never mix degrees and radians in code — convert at the system boundary (e.g. mission file parsing) only.

## Coordinate frames

- **World-fixed planning frame: `map`, ENU** (East-North-Up). All World Model, Planning, and Mission data is expressed here.
- **Body frame: `base_link`, FLU** (Forward-Left-Up), attached to the vehicle.
- **Sensor frames:** `lidar_link`, `imu_link`, each with a static `tf2` transform to `base_link`, published by the Localization module at startup.
- **PX4 boundary:** PX4 internally uses NED. The **PX4 Interface module is the only place** ENU↔NED conversion happens. No other module may assume NED.

## Timestamps

- Every message header uses `header.stamp` = the time the underlying measurement/estimate was valid (sensor time), not publish time.
- All nodes use ROS time (`use_sim_time` toggled by launch config — `true` under Gazebo/SITL, `false` on hardware bench tests).
- Timestamp synchronization between LiDAR and IMU is Person 1's responsibility and must be resolved before fusion, not downstream.

## Quaternion convention

`geometry_msgs/Quaternion` (x, y, z, w), Hamilton convention, right-handed, matching ROS 2 / Eigen defaults. Always normalize before publishing.

## Map resolution

Default voxel resolution: **0.2 m**. Any module changing this must update it in `config/` and notify Planning + World Model owners (it affects planning cost lookups).

## Topic naming

`/<module>/<data>`, snake_case, no trailing slash:

| Topic | Type | Producer |
|---|---|---|
| `/localization/state` | `LocalizationState` | Localization |
| `/world_model/local_map` | `LocalMap` | World Model |
| `/world_model/obstacles` | `ObstacleSet` | World Model |
| `/mission/current` | `Mission` | Mission Manager |
| `/planning/trajectory` | `Trajectory` | Planning |
| `/planning/status` | `PlannerStatus` | Planning |
| `/safety/vehicle_command` | `VehicleCommand` | Safety Supervisor |
| `/safety/system_health` | `SystemHealth` | Safety Supervisor |

## Service / action naming

`/<module>/<verb_noun>`, e.g. `/mission/load_mission` (service), `/planning/execute_mission` (action). None exist yet in the mock phase — add here when introduced, PR review required.

## QoS rules

- **High-rate sensor/state data** (LocalizationState, LocalMap, ObstacleSet, Trajectory, PlannerStatus, SystemHealth, VehicleCommand): `BEST_EFFORT`, `KEEP_LAST` depth 5. Losing one sample is fine; latency matters more than delivery guarantee.
- **Mission** (low-rate, must-not-miss, must-be-available-to-late-joiners): `RELIABLE`, `TRANSIENT_LOCAL`, `KEEP_LAST` depth 1.

## Error / status conventions

- Every module that can fail publishes health via `SystemHealth` (owned by Safety) or embeds a status/validity field in its own message (`localization_ok`, `map_valid`, `valid` on `Trajectory`/`VehicleCommand`).
- A `false` validity flag means **downstream must not act on the payload**, only on the fact that it's invalid. Never infer validity from field values (e.g. all-zero position ≠ invalid).
- Log levels: `debug` for per-tick internals, `info` for state transitions, `warn` for degraded-but-continuing, `error` for a module that cannot continue safely.

## No local invention

If a convention isn't listed here, raise it in a PR that updates this file first — don't encode a private assumption in one module's code.
