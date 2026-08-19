# Localization Module

Owner: Person 1. This documents the real Localization implementation, layered on top of the frozen `LocalizationState` contract in [docs/INTERFACES.md](INTERFACES.md). `MockLocalization` still exists and still runs — the rest of the team keeps developing against it; this doesn't replace it, it sits alongside it as an alternative producer of the same topic.

## Status

**Scaffolded, not yet running end-to-end.** What exists now:

- `lio_state_bridge` (C++, builds and lints clean) — the adapter node, described below.
- `real_localization.launch.py`, `fast_lio_x500.yaml`, static TF publishers for `lidar_link`/`imu_link`.
- `uav_localization.repos` — vendoring pointer for the LIO backend.
- `ros_gz_bridge_lidar_imu.yaml` + `simulation/launch/sensors_bridge.launch.py`.

What's **not** done, and blocks actually running this:

- No LiDAR/IMU-equipped Gazebo vehicle model exists yet. The bridge config and launch files assume a model named `x500_lidar` with sensors at specific link paths — that's a placeholder, not a built model. Coordinate with Person 4 (owns Gazebo/PX4 integration) on where this model should live and what it should actually be called.
- `extrinsic_T`/`extrinsic_R` in `fast_lio_x500.yaml` are identity placeholders. They must reflect the real (or simulated) LiDAR->IMU mount transform or the odometry will be visibly wrong even if everything else works.
- The vendored `fast_lio` repo URL in `uav_localization.repos` is a best-guess at a maintained ROS 2 port; I could not verify it builds against Jazzy from this environment (no network/ROS access here). **First task: confirm it builds, or swap in whichever ROS 2 FAST-LIO2 fork you land on**, and update this doc.
- `lio_state_bridge`'s confidence/status heuristic (see below) is a stand-in. It's timestamp-staleness-only — it doesn't look at anything FAST-LIO2 exposes about registration quality/degeneracy. Real localization-health work is improving this, not just wiring topics.

## Why this design

Every other module (World Model, Planning, Safety) was built and tested against `MockLocalization` publishing on `/localization/state`. That contract doesn't change here — `lio_state_bridge` publishes the exact same `LocalizationState` message on the exact same topic. Swapping mock for real is a launch-file change (`mock_pipeline.launch.py` -> `real_localization_pipeline.launch.py`), not an interface change. See `docs/DEVELOPMENT.md` for the general "develop against mocks, swap in real later" workflow this is an instance of.

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

`lidar_link` and `imu_link` are published as static transforms from `base_link` (see `real_localization.launch.py`) per the sensor-frame convention in `docs/CONVENTIONS.md`.

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

# bring up the sim sensors (once the vehicle model exists — see Status above)
ros2 launch $(pwd)/simulation/launch/sensors_bridge.launch.py

# then, in place of mock_pipeline.launch.py:
ros2 launch uav_bringup real_localization_pipeline.launch.py
```

Verify the same way you'd verify the mock: `ros2 topic echo /localization/state` should show `localization_ok: true` and a `status` that tracks whether FAST-LIO2 is actually publishing.

## Next tasks, roughly in order

1. Get the vendored FAST-LIO2 fork building against ROS 2 Jazzy; fix `uav_localization.repos` and `real_localization.launch.py`'s assumed launch/topic names to match whatever actually works.
2. With Person 4: define and build the LiDAR/IMU-equipped Gazebo vehicle model; update `ros_gz_bridge_lidar_imu.yaml`'s `gz_topic_name`s and `fast_lio_x500.yaml`'s `extrinsic_T`/`extrinsic_R` to match.
3. Timestamp synchronization between the LiDAR and IMU sources (`docs/CONVENTIONS.md` calls this out as your responsibility) — confirm the sim sensors are already synced or add correction.
4. Add contract tests for `lio_state_bridge` itself (currently only `MockLocalization`'s output is contract-tested) — same pattern as `tests/contract/test_node_contracts.py`, publishing synthetic `Odometry` and asserting the staleness/status table above.
5. Improve the confidence heuristic past "is it fresh" — look at what the vendored backend actually exposes about registration quality (e.g. FAST-LIO2's ESKF covariance) instead of a fixed 0.9/0.3/0.0.
6. Run the golden scenario (`docs/TESTING.md`) with real localization once the above lands, and compare drift against the mocked baseline.
