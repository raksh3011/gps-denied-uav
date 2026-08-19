# Interfaces

All definitions live in `src/interfaces/uav_interfaces/msg/`. This file documents intent; the `.msg` file is the source of truth for fields — keep both in sync in the same PR.

## LocalizationState

- **Producer:** Localization (Person 1)
- **Consumers:** World Model, Planning, Safety, Mission
- **Frame:** `map` (ENU)
- **Rate:** 100-200 Hz
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Validity:** consumers must check `localization_ok` and `status`; do not use `pose`/`twist` when `localization_ok == false`.
- **Fields:** pose (position+orientation), twist (body-frame velocity), 6x6 covariances, confidence [0,1], status enum.

## LocalMap

- **Producer:** World Model (Person 2)
- **Consumers:** Planning, Safety
- **Frame:** `map` (ENU)
- **Rate:** 5-10 Hz
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Validity:** check `map_valid`; check `header.stamp` age (>1s = stale, use with caution).
- **Fields:** resolution (m/voxel), origin, size_x/y/z, flat `occupancy[]` (0=free,1=occupied,255=unknown).

## ObstacleSet

- **Producer:** World Model (Person 2)
- **Consumers:** Planning, Safety
- **Frame:** `map` (ENU)
- **Rate:** 5-10 Hz (paired with LocalMap)
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Validity:** empty list is valid (means "no obstacles seen"), not an error.
- **Fields:** array of `Obstacle` (id, position, velocity, radius, class).

## Mission

- **Producer:** Mission Manager (Person 4)
- **Consumers:** Global Planner (Person 3)
- **Frame:** `map` (ENU)
- **Rate:** on-change only
- **QoS:** RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(1) — late-joining Planner must still receive the last mission.
- **Validity:** `waypoints` must be non-empty for a Planner to act.
- **Fields:** mission_id, ordered `Waypoint[]`, max_speed, boundary_radius (geofence), min/max_altitude.

## Trajectory

- **Producer:** Local Planner (Person 3)
- **Consumers:** Safety, (indirectly) PX4 Interface
- **Frame:** `map` (ENU)
- **Rate:** 10-20 Hz
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Validity:** `valid` must be true; an invalid Trajectory must never be forwarded toward PX4. Safety is the enforcement point.
- **Fields:** time-parameterized `TrajectoryPoint[]` (position, velocity, acceleration, yaw).

## PlannerStatus

- **Producer:** Planning (Person 3)
- **Consumers:** Mission Manager, Safety
- **Rate:** matches Trajectory (10-20 Hz)
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Fields:** state enum (IDLE/PLANNING/EXECUTING/REPLANNING/GOAL_REACHED/FAILED), message, progress [0,1].

## SystemHealth

- **Producer:** Safety Supervisor (Person 4)
- **Consumers:** Mission Manager, Ground Station (telemetry, non-blocking)
- **Rate:** 5-10 Hz
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Fields:** overall_level + per-subsystem levels (OK/WARN/CRITICAL/FAILSAFE), active_faults[] (short string codes).

## VehicleCommand

- **Producer:** Safety Supervisor (Person 4)
- **Consumers:** PX4 Interface -> PX4
- **Frame:** position setpoints in `map` (ENU); converted to NED only inside PX4 Interface.
- **Rate:** >= 2 Hz required by PX4 offboard mode, target 20-50 Hz.
- **QoS:** BEST_EFFORT, KEEP_LAST(5)
- **Validity:** `valid == false` means PX4 Interface must reject and hold the last safe command — never partially act on a rejected command's fields.
- **Fields:** mode enum (POSITION/VELOCITY/LAND/RTL/HOLD/DISARM), position, velocity, yaw, valid.

## Adding or changing an interface

1. Open a PR editing the `.msg` file **and** this doc together.
2. Tag all four team members as reviewers — interface changes require team approval (see [GIT_WORKFLOW.md](GIT_WORKFLOW.md)).
3. Update any mock producer/consumer in the same PR so `colcon build` and contract tests still pass.
