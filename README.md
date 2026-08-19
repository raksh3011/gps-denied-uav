# GPS-Denied Autonomous UAV Navigation Stack

Modular, low-latency autonomous UAV software stack using LiDAR + IMU as primary sensing, for GPS-denied environments. Built by a 4-person team on Windows PCs via WSL2 + Ubuntu 24.04 + ROS 2 Jazzy + PX4 SITL.

**Status: Initialization phase.** No real Localization/World Model/Planning/Safety algorithms exist yet — see [Definition of Done](#definition-of-done) and [First Milestone](#first-milestone) below. Everything currently in `src/` is either a frozen interface or a mock.

## Mission flow

```
Mission Input -> Global Planning -> GPS-denied Localization -> Local World Model
  -> Dynamic Obstacle Avoidance -> Local Replanning -> Safety Monitoring
  -> Autonomous Mission Execution -> Target -> Return -> Landing
```

The system does not depend on continuous Ground Station control during mission execution. Full architecture: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## 1. Setup steps (all four Windows PCs)

Run on Windows (PowerShell, Administrator):

```powershell
wsl --install -d Ubuntu-24.04
wsl --set-default-version 2
```

Then inside WSL2 Ubuntu 24.04:

```bash
cd ~
git clone <REPO_URL> gps-denied-uav
cd gps-denied-uav
chmod +x setup/*.sh
./setup/install_dependencies.sh
./setup/verify_environment.sh
./setup/setup_workspace.sh
source install/setup.bash
```

Open VS Code with the WSL extension via `code .` from inside the WSL2 shell. Full detail: [setup/README.md](setup/README.md).

## 2. Repository tree

```
gps-denied-uav/
├── src/
│   ├── interfaces/uav_interfaces/     # frozen .msg contracts (team-approved changes only)
│   ├── localization/uav_localization/ # Person 1 — mock_localization node
│   ├── world_model/uav_world_model/   # Person 2 — mock_world_model node
│   ├── planning/uav_planning/         # Person 3 — mock_planner node
│   ├── mission/uav_mission/           # Person 4 — mock_mission node
│   ├── safety/uav_safety/             # Person 4 — mock_safety node
│   ├── vehicle/uav_vehicle/           # Person 4 — mock_vehicle (PX4 interface) node
│   └── simulation/uav_bringup/        # Person 4 — system launch files
│
├── tests/
│   ├── unit/            # pure-logic tests
│   ├── contract/        # interface/message + node contract tests
│   └── integration/     # full mocked-pipeline test
│
├── simulation/
│   ├── worlds/ models/ scenarios/ launch/   # Gazebo/PX4 SITL assets (populated post-Milestone-1)
│
├── config/               # shared runtime config
├── launch/               # top-level launch entry points (delegates to uav_bringup)
├── scripts/               # dev utility scripts
├── setup/                 # environment install/verify scripts
├── docs/                  # ARCHITECTURE, CONVENTIONS, INTERFACES, DEVELOPMENT, TESTING, GIT_WORKFLOW, TEAM_OWNERSHIP, LOCALIZATION
├── pytest.ini
├── uav_localization.repos # vcs-vendored external deps (e.g. FAST-LIO2) — see docs/DEVELOPMENT.md
├── .github/workflows/ci.yml
└── README.md
```

## 3. Team ownership

| Person | Owns | Provides | Consumes |
|---|---|---|---|
| 1 — Localization | LiDAR/IMU interface, sync, LIO, pose/vel/covariance, health | `LocalizationState` | raw sensors (mocked) |
| 2 — World Model | LiDAR preprocessing, rolling map, occupancy, obstacles | `LocalMap`, `ObstacleSet` | `LocalizationState` |
| 3 — Planning | global/local planner, cost function, replanning, boundary check | `Trajectory`, `PlannerStatus` | `Mission`, `LocalizationState`, `LocalMap`, `ObstacleSet` |
| 4 — Mission/Safety/Integration | mission state machine, safety supervisor, watchdogs, PX4 interface, launch/CI | `Mission`, `SystemHealth`, `VehicleCommand` | `PlannerStatus`, `Trajectory`, `LocalizationState` |

Full detail: [docs/TEAM_OWNERSHIP.md](docs/TEAM_OWNERSHIP.md).

## 4. Interfaces

| Interface | Producer | Consumers | Frame | Rate |
|---|---|---|---|---|
| `LocalizationState` | Localization | World Model, Planning, Safety, Mission | map (ENU) | 100-200 Hz |
| `LocalMap` | World Model | Planning, Safety | map (ENU) | 5-10 Hz |
| `ObstacleSet` | World Model | Planning, Safety | map (ENU) | 5-10 Hz |
| `Mission` | Mission Manager | Global Planner | map (ENU) | on-change |
| `Trajectory` | Planning | Safety | map (ENU) | 10-20 Hz |
| `PlannerStatus` | Planning | Mission Manager, Safety | — | 10-20 Hz |
| `SystemHealth` | Safety Supervisor | Mission Manager, Ground Station | — | 5-10 Hz |
| `VehicleCommand` | Safety Supervisor | PX4 Interface | map->NED at boundary | >=2 Hz |

Full field-level detail: [docs/INTERFACES.md](docs/INTERFACES.md). Conventions (units, frames, QoS, naming): [docs/CONVENTIONS.md](docs/CONVENTIONS.md).

## 5. Git workflow

```
main <- develop <- feature/localization
                 <- feature/world-model
                 <- feature/planning
                 <- feature/mission-safety
```

No direct pushes to `main`. Every change via PR into `develop`, reviewed, CI-green. Interface changes require all-four approval. Full detail: [docs/GIT_WORKFLOW.md](docs/GIT_WORKFLOW.md).

## 6. Build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch uav_bringup mock_pipeline.launch.xml
```

Inspect the running pipeline:

```bash
ros2 topic list
ros2 topic echo /safety/vehicle_command
```

## 7. Run tests

```bash
colcon test && colcon test-result --verbose   # package-level
pytest tests/ -v                              # unit + contract + integration
```

## 8. Definition of Done

A module is DONE only if it:

- Builds from a clean clone
- Follows the interface contract ([docs/INTERFACES.md](docs/INTERFACES.md))
- Passes unit tests, contract tests, module tests, integration tests
- Has no machine-specific paths
- Has documentation and a README
- Has a test/demo command
- Passes CI
- Has been reviewed by another team member

## 9. First milestone

All four developers can: clone the repo, start WSL2, build the workspace, launch the mocked pipeline, see all modules communicate correctly, run contract/unit tests, run CI, run the baseline simulation, create a feature branch, and open/merge a PR successfully — **before** any real Localization/World Model/Planning/Mission-Safety algorithm work begins.

## 10. First-week task breakdown

**All four (day 1):**
- Complete [setup/README.md](setup/README.md) on your own PC, confirm `verify_environment.sh` is all-PASS.
- Clone, build, `ros2 launch uav_bringup mock_pipeline.launch.xml`, confirm `/safety/vehicle_command` is publishing.
- Run `pytest tests/ -v`, confirm green.
- Practice the PR flow once on a throwaway doc change into your feature branch -> `develop`.

**Person 1 (Localization):**
- Read `LocalizationState` contract in [docs/INTERFACES.md](docs/INTERFACES.md).
- Stand up `lidar_link`/`imu_link` static TF stubs and a `MockLiDAR`/`MockIMU` data source.
- Sketch the timestamp-sync approach (design note in `docs/`, no code yet) for review.

**Person 2 (World Model):**
- Read `LocalMap`/`ObstacleSet` contracts.
- Replace the mock's fixed obstacle with a config-driven list (still mocked) to unblock Planning's boundary-check testing.
- Decide voxel occupancy library approach (plain array vs. PCL octree) — write a short tradeoff note.

**Person 3 (Planning):**
- Read `Mission`/`Trajectory`/`PlannerStatus` contracts.
- Develop against `MockLocalization` + `MockWorldModel` + `MockMission`; start global-planner cost-function design doc.
- Identify what "boundary checking" needs from `Mission.boundary_radius` and `min/max_altitude`.

**Person 4 (Mission/Safety/Integration):**
- Own CI green on `develop`; fix any environment gaps `verify_environment.sh` surfaces across the other 3 PCs.
- Flesh out the mission state machine design (still using `MockPlanner` underneath).
- Start Gazebo world + PX4 SITL smoke test (empty world, arm/takeoff/land) independent of the ROS 2 mock pipeline, toward the golden scenario.

## Setup problems discovered and fixes

See the table in [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md#common-setup-problems) — kept up to date as issues surface across the four machines (e.g. `ros2` not found until reshelling, `uav_interfaces` Python bindings needing a rebuild before import, WSL2 clock drift, first PX4 SITL build being slow).
