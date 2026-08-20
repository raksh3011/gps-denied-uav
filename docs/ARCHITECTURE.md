# Architecture

## Mission flow

```
Mission Input
  -> Global Planning
  -> GPS-denied Localization
  -> Local World Model
  -> Dynamic Obstacle Avoidance
  -> Local Replanning
  -> Safety Monitoring
  -> Autonomous Mission Execution
  -> Target
  -> Return
  -> Landing
```

## Node graph (current: mocked)

```
Mission Manager (uav_mission)
      |  /mission/current
      v
Global/Local Planner (uav_planning)  <-- /localization/state (uav_localization)
      |  /planning/trajectory              /world_model/local_map, /world_model/obstacles (uav_world_model)
      |  /planning/status
      v
Safety Supervisor (uav_safety)  <-- /localization/state, /planning/trajectory
      |  /safety/vehicle_command
      |  /safety/system_health
      v
PX4 Interface (uav_vehicle)
      v
     PX4
```

Safety Supervisor is the **only** path into the vehicle. Planning never talks to the vehicle directly, and the vehicle interface never trusts a Trajectory that didn't pass through Safety's `valid` gate. This is deliberate: it keeps a single, auditable choke point for "is it safe to move the vehicle."

## Package -> team mapping

| Package | Owner | Milestone-1 content |
|---|---|---|
| `interfaces/uav_interfaces` | shared, team-approved | all `.msg` definitions |
| `localization/uav_localization` | Person 1 | `mock_localization` node |
| `world_model/uav_world_model` | Person 2 | `mock_world_model` node |
| `planning/uav_planning` | Person 3 | `mock_planner` node |
| `mission/uav_mission` | Person 4 | `mock_mission` node |
| `safety/uav_safety` | Person 4 | `mock_safety` node |
| `vehicle/uav_vehicle` | Person 4 | `mock_vehicle` node |
| `simulation/uav_bringup` | Person 4 | system launch files |

## Ground-station independence

Mission Manager loads a mission once (from file, or a one-shot ground-station push) and the rest of the loop — Planner, Localization, World Model, Safety, Vehicle Interface — runs closed-loop with no requirement for continued ground-station connectivity. `SystemHealth` is broadcast for telemetry only; nothing downstream blocks on it being received.

## Real modules, as they land

Real implementations plug into the exact topics/messages their mock used, alongside — not replacing — the mock:
- Localization: LiDAR/IMU -> FAST-LIO2 -> `lio_state_bridge` -> `LocalizationState`. See [docs/LOCALIZATION.md](LOCALIZATION.md).
- Planning: `Mission`/`LocalizationState`/`LocalMap`/`ObstacleSet` -> Theta* global + D* Lite local (with confidence-adaptive risk margin) + boundary checking + trajectory generation (`real_planner`) -> `Trajectory`/`PlannerStatus`. See [docs/PLANNING.md](PLANNING.md).
- World Model: point cloud (`/cloud_registered`) + `LocalizationState` -> rolling voxel occupancy window + obstacle clustering/tracking (`real_world_model`) -> `LocalMap`/`ObstacleSet`. See [docs/WORLD_MODEL.md](WORLD_MODEL.md).
- Safety: `LocalizationState`/`Trajectory`/`LocalMap`/`ObstacleSet`/`PlannerStatus` -> staleness/validity gating + independent obstacle-clearance check + sustained-loss escalation (`real_safety`) -> `VehicleCommand`/`SystemHealth`. See [docs/SAFETY.md](SAFETY.md).

## Why mocks first

Every arrow in the diagram above is a `.msg` contract. Milestone 1 proves every arrow works — wrong units, wrong frame, missing validity flag, wrong QoS — **before** anyone spends weeks on LIO/SLAM/planning algorithms that would otherwise surface these bugs late and expensively. See [TEAM_OWNERSHIP.md](TEAM_OWNERSHIP.md) for how each person develops independently against mocks of their dependencies.
