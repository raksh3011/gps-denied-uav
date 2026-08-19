# Localization Module

Owner: Person 1. This documents the real Localization implementation, layered on top of the frozen `LocalizationState` contract in [docs/INTERFACES.md](INTERFACES.md). `MockLocalization` still exists and still runs — the rest of the team keeps developing against it; this doesn't replace it, it sits alongside it as an alternative producer of the same topic.

## Status

**Scaffolded and internally consistent, not yet verified end-to-end** (no ROS 2/Gazebo toolchain in the environment this was built in — see [Verifying this on your machine](#verifying-this-on-your-machine)). What exists now:

- `lio_state_bridge` (C++, builds and lints clean) — the adapter node, described below.
- `simulation/models/x500_lidar/` — a real Gazebo model: PX4's stock `x500` plus a `gpu_lidar` on `lidar_link` and an `imu` sensor on `imu_link`, both fixed-jointed to `base_link`. This is the source of truth for the sensor mount, not a placeholder.
- `real_localization.launch.xml`, `fast_lio_x500.yaml`, static TF publishers for `lidar_link`/`imu_link` — extrinsics (`extrinsic_T`/`extrinsic_R` in the yaml, the TF publisher `args` in the launch file) are read directly off `model.sdf`, not guessed. If the mount ever moves, update all three together — that's now the single place drift between "what the sim actually has" and "what FAST-LIO2 is told" can creep in.
- `uav_localization.repos` — vendoring pointer for the LIO backend.
- `ros_gz_bridge_lidar_imu.yaml` + `simulation/launch/sensors_bridge.launch.xml`, topic names matching `model.sdf`'s sensor `<topic>` tags.

What's **not** done, and blocks actually running this:

- The vendored `fast_lio` repo URL in `uav_localization.repos` is a best-guess at a maintained ROS 2 port; I could not verify it builds against Jazzy from this environment (no network/ROS access here). **First task: confirm it builds, or swap in whichever ROS 2 FAST-LIO2 fork you land on**, and update this doc plus `real_localization.launch.xml`'s `<include>` if the package/launch-file name differs.
- `x500_lidar/model.sdf` has never been loaded into Gazebo — it's written to the SDF 1.9 spec and to the same pattern PX4's own sensor-variant models (e.g. `x500_depth`) use, but a syntax slip is possible. First thing to check if Gazebo rejects it.
- `PX4_GZ_MODEL=x500_lidar` requires the model to be discoverable — either copy `simulation/models/x500_lidar/` into `PX4-Autopilot/Tools/simulation/gz/models/`, or add this repo's `simulation/models` to `GZ_SIM_RESOURCE_PATH`. Not automated yet; a `setup/` script for this is a reasonable next addition.
- `acc_cov`/`gyr_cov`/etc. in `fast_lio_x500.yaml` are FAST-LIO2's stock defaults, not tuned against the simulated IMU's actual noise characteristics.
- `lio_state_bridge`'s confidence/status heuristic (see below) is a stand-in. It's timestamp-staleness-only — it doesn't look at anything FAST-LIO2 exposes about registration quality/degeneracy. Real localization-health work is improving this, not just wiring topics.

## Why this design

Every other module (World Model, Planning, Safety) was built and tested against `MockLocalization` publishing on `/localization/state`. That contract doesn't change here — `lio_state_bridge` publishes the exact same `LocalizationState` message on the exact same topic. Swapping mock for real is a launch-file change (`mock_pipeline.launch.xml` -> `real_localization_pipeline.launch.xml`), not an interface change. See `docs/DEVELOPMENT.md` for the general "develop against mocks, swap in real later" workflow this is an instance of.

The actual state estimation is FAST-LIO2, not code we wrote. `lio_state_bridge` is deliberately thin: it does not do sensor fusion, it converts FAST-LIO2's `nav_msgs/Odometry` output into our contract and turns silence into an honest `localization_ok=false` instead of letting stale data look live. Keeping the adapter thin means swapping the backend later (a different LIO algorithm, or real hardware with a different driver) only touches this one file plus the launch file — not any downstream module.

## License boundary — read before touching this code

FAST-LIO2 is **GPLv2**. Everything else in this repo is Apache-2.0. That's fine as long as the boundary stays a process boundary: `lio_state_bridge` talks to FAST-LIO2 only over ROS topics (DDS/IPC), it never `#include`s FAST-LIO2 headers or links against it. Do not "simplify" this by merging the bridge logic into FAST-LIO2's own node or vice versa — that would pull GPLv2 obligations onto the rest of the stack. If a future backend swap considers a permissively-licensed alternative (e.g. an ICP/NDT-based estimator we write ourselves), that removes this constraint entirely; note it here if that happens.

## Architecture

```
/lidar/points (sensor_msgs/PointCloud2)  ─┐
/imu/data (sensor_msgs/Imu)              ─┼─> FAST-LIO2 ──> /Odometry (nav_msgs/Odometry)
                                          ─┘                      │
                                                                   v
                                                        lio_state_bridge
                                                                   │
                                                                   v
                                                  /localization/state (LocalizationState)
```

`lidar_link` and `imu_link` are published as static transforms from `base_link` (see `real_localization.launch.xml`) per the sensor-frame convention in `docs/CONVENTIONS.md`.

## Confidence / status heuristic (v1)

`lio_state_bridge` tracks time since the last `/Odometry` message:

| Age since last odometry | `status` | `confidence` | `localization_ok` |
|---|---|---|---|
| `<= staleness_timeout_s` (default 0.3s) | `STATUS_NOMINAL` | 0.9 | `true` |
| `<= lost_timeout_s` (default 1.5s) | `STATUS_DEGRADED` | 0.3 | `false` |
| beyond that, or no odometry ever received | `STATUS_LOST` | 0.0 | `false` |

This is intentionally conservative: `localization_ok` only goes `true` when data is fresh, full stop. It says nothing about whether FAST-LIO2's estimate is actually *good* while it's fresh — that's the next real piece of work (see below).

## Build and run

```bash
# one-time: vendor the LIO backend
vcs import src < uav_localization.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# make the model discoverable (one-time; see Status above for alternatives)
cp -r simulation/models/x500_lidar "$HOME/PX4-Autopilot/Tools/simulation/gz/models/"

# terminal 1: Gazebo + PX4 SITL with the sensor-equipped vehicle
cd ~/PX4-Autopilot && PX4_GZ_MODEL=x500_lidar make px4_sitl gz_x500

# terminal 2: bridge the simulated sensors into ROS 2
ros2 launch $(pwd)/simulation/launch/sensors_bridge.launch.xml

# terminal 3: in place of mock_pipeline.launch.xml
ros2 launch uav_bringup real_localization_pipeline.launch.xml
```

## Verifying this on your machine

Work through these in order — each one isolates a different layer, so if something's wrong you'll know which piece before moving to the next:

1. **The model loads.** After `make px4_sitl gz_x500` with `PX4_GZ_MODEL=x500_lidar`, Gazebo should show the quad with a small cylinder (the LiDAR) on top. If it silently falls back to the plain `x500`, the model wasn't found — check the copy step above, or `echo $GZ_SIM_RESOURCE_PATH`.
2. **Sensors are actually publishing in Gazebo**, before touching ROS 2 at all:
   ```bash
   gz topic -l | grep -E 'lidar|imu'
   gz topic -e -t /world/default/model/x500_lidar/link/imu_link/sensor/imu/imu
   ```
   If these don't list/echo, the SDF sensor tags are wrong — fix `model.sdf` before going further; the ROS bridge can't produce data Gazebo isn't producing.
3. **The ROS 2 bridge is relaying it**, with `sensors_bridge.launch.xml` running:
   ```bash
   ros2 topic hz /lidar/points     # expect ~10 Hz, matching model.sdf's lidar update_rate
   ros2 topic hz /imu/data         # expect ~200 Hz
   ros2 topic echo /imu/data --once
   ```
4. **FAST-LIO2 is producing odometry**, with the full `real_localization.launch.xml` running:
   ```bash
   ros2 topic hz /Odometry
   ros2 topic echo /Odometry --once
   ```
   If `/lidar/points` and `/imu/data` are both flowing but `/Odometry` never appears, the problem is inside the vendored FAST-LIO2 config — check its own log output first (`fast_lio_x500.yaml`'s topic names, `lidar_type`/`timestamp_unit` matching the simulated sensor).
5. **`lio_state_bridge` is relaying it correctly** into our contract:
   ```bash
   ros2 topic echo /localization/state
   ```
   Fly the vehicle a short distance (even a manual RC/QGroundControl takeoff) and confirm: `localization_ok: true`, `status: 0` (`STATUS_NOMINAL`), `pose.position` changing plausibly with movement, `header.frame_id: map`. Stop `/Odometry` (kill the FAST-LIO2 process) and confirm `/localization/state` transitions `NOMINAL -> DEGRADED -> LOST` over ~0.3s and ~1.5s respectively, matching the table below — that failure-path behavior matters as much as the happy path, since it's what Safety depends on.
6. **The full pipeline still holds together.** With `real_localization_pipeline.launch.xml` running end-to-end, `ros2 topic echo /safety/vehicle_command` should behave exactly like it did with `MockLocalization` in the Milestone 1 verification — `valid: true` once localization is healthy, `MODE_HOLD`/`valid: false` if you kill localization. If Safety's behavior differs at all between mock and real Localization, that's a contract violation somewhere in this chain, not a Safety bug — `LocalizationState` is supposed to be indistinguishable to downstream consumers regardless of producer.

Once (5) and (6) pass, this module has cleared the same bar `MockLocalization` already cleared — see [docs/TESTING.md](TESTING.md) for what "contract-tested" means and task 4 below for making that automatic rather than manual.

## Next tasks, roughly in order

1. Get the vendored FAST-LIO2 fork building against ROS 2 Jazzy; fix `uav_localization.repos` and `real_localization.launch.xml`'s assumed launch/topic names to match whatever actually works.
2. Confirm `x500_lidar/model.sdf` loads in Gazebo and its sensors publish (steps 1-2 above); fix any SDF syntax issues.
3. Timestamp synchronization between the LiDAR and IMU sources (`docs/CONVENTIONS.md` calls this out as your responsibility) — confirm the sim sensors are already synced or add correction.
4. Add contract tests for `lio_state_bridge` itself (currently only `MockLocalization`'s output is contract-tested) — same pattern as `tests/contract/test_node_contracts.py`, publishing synthetic `Odometry` and asserting the staleness/status table above, so step 5's manual check above becomes a `pytest` assertion.
5. Improve the confidence heuristic past "is it fresh" — look at what the vendored backend actually exposes about registration quality (e.g. FAST-LIO2's ESKF covariance) instead of a fixed 0.9/0.3/0.0.
6. Tune `acc_cov`/`gyr_cov`/etc. in `fast_lio_x500.yaml` against the simulated IMU's actual noise, instead of FAST-LIO2's stock defaults.
7. Run the golden scenario (`docs/TESTING.md`) with real localization once the above lands, and compare drift against the mocked baseline.
