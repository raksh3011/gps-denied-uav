# Development Guide

## First-time setup

See [setup/README.md](../setup/README.md). Summary:

```bash
git clone <REPO_URL> gps-denied-uav && cd gps-denied-uav
chmod +x setup/*.sh
./setup/install_dependencies.sh
./setup/verify_environment.sh
./setup/setup_workspace.sh
source install/setup.bash
```

## Daily workflow

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash          # after any build
colcon build --symlink-install     # rebuild after changes
colcon test                        # ROS-level package tests
pytest tests/ -v                   # unit/contract/integration tests
```

`--symlink-install` means Python file edits (mock nodes, launch files) take effect without rebuilding; C++ changes still need a rebuild.

## Running the mocked pipeline

```bash
ros2 launch uav_bringup mock_pipeline.launch.py
```

In another terminal, inspect topics:

```bash
ros2 topic list
ros2 topic echo /safety/vehicle_command
ros2 topic hz /planning/trajectory
```

## Developing against a dependency that isn't ready

Use the corresponding mock instead of waiting. Example: Person 3 developing the real local planner against a not-yet-real localization:

```bash
ros2 run uav_localization mock_localization &
ros2 run uav_world_model mock_world_model &
ros2 run uav_mission mock_mission &
ros2 run uav_planning <your_real_planner_node>
```

Swap in the real node for a module as soon as it passes its own contract tests — the topic/message contract means nothing downstream needs to change.

## Common setup problems

| Symptom | Fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/jazzy/setup.bash`, or open a new shell if `setup_workspace.sh` already added it to `~/.bashrc` |
| `colcon build` can't find `uav_interfaces` msgs in Python | rebuild (`colcon build --packages-select uav_interfaces`) then re-source `install/setup.bash` — generated Python bindings only appear after a build |
| WSL2 clock drift causing weird ROS timestamps | `sudo hwclock -s` inside WSL2, or restart WSL2 (`wsl --shutdown` from PowerShell) |
| `rosdep install` fails on an unknown key | run `rosdep update` first; if still failing, the package name may need adding to `/etc/ros/rosdep/sources.list.d/` — ask before hand-patching, file a note in this table |
| PX4 SITL build very slow first time | expected — first `make px4_sitl` build compiles NuttX/toolchain deps, ~10-20 min on a typical laptop |
| VS Code shows red squiggles on ROS headers | make sure you opened the folder via `code .` from inside WSL2, not from Windows Explorer / a Windows path |

## C++ vs Python

Core real-time modules (LIO, planning, safety-critical control loops) are **C++20**. Mocks, tooling, and test harnesses are Python — this is intentional to keep Milestone 1 fast to iterate on; do not port mocks to C++.
