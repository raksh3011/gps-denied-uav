# Localization Module

Owner: Person 1. This documents the real Localization implementation, layered on top of the frozen `LocalizationState` contract in [docs/INTERFACES.md](INTERFACES.md). `MockLocalization` still exists and still runs — the rest of the team keeps developing against it; this doesn't replace it, it sits alongside it as an alternative producer of the same topic.

## Status

**Confirmed working end-to-end on real hardware (2026-08-19, raksh's WSL2 machine), through PX4 SITL reaching a flyable `pxh>` prompt with our sensors attached.** Everything in [Build and run](#build-and-run) below is the exact sequence that got there — every step in it was a real failure encountered and fixed, not a guess. What's still open:

- The vendored FAST-LIO2 backend (`fast_lio` + `livox_ros_driver2`) builds clean, but hasn't yet been run against the live `/lidar/points`/`/imu/data` topics to confirm it actually produces `/Odometry`. That's the next real milestone — see [Next tasks](#next-tasks-roughly-in-order).
- `acc_cov`/`gyr_cov`/etc. in `fast_lio_x500.yaml` are FAST-LIO2's stock defaults, not tuned against the simulated IMU's actual noise characteristics.
- `lio_state_bridge`'s confidence/status heuristic (see below) is a stand-in — timestamp-staleness-only, doesn't look at anything FAST-LIO2 exposes about registration quality/degeneracy.

## Why this design

Every other module (World Model, Planning, Safety) was built and tested against `MockLocalization` publishing on `/localization/state`. That contract doesn't change here — `lio_state_bridge` publishes the exact same `LocalizationState` message on the exact same topic. Swapping mock for real is a launch-file change (`mock_pipeline.launch.xml` -> `real_localization_pipeline.launch.xml`), not an interface change. See `docs/DEVELOPMENT.md` for the general "develop against mocks, swap in real later" workflow this is an instance of.

The actual state estimation is FAST-LIO2, not code we wrote. `lio_state_bridge` is deliberately thin: it does not do sensor fusion, it converts FAST-LIO2's `nav_msgs/Odometry` output into our contract and turns silence into an honest `localization_ok=false` instead of letting stale data look live. Keeping the adapter thin means swapping the backend later (a different LIO algorithm, or real hardware with a different driver) only touches this one file plus the launch file — not any downstream module.

## Why there's no separate "x500_lidar" model

The original plan was a distinct Gazebo model (`x500_lidar`) alongside PX4's stock `x500`. That turned out to not be practical: PX4's SITL launch plumbing — the `make px4_sitl gz_x500` Ninja target, and `PX4_SIM_MODEL`-based airframe auto-detection — is hardcoded to a small, fixed set of known model names. We spent a long debugging session confirming:

- `PX4_GZ_MODEL` (an env var we initially guessed at) isn't read anywhere in this PX4 version at all.
- The real variable is `PX4_SIM_MODEL` (format `gz_<model>`), but the `gz_x500` Ninja target hardcodes `PX4_SIM_MODEL=gz_x500` as an inline command-line assignment, which always wins over anything exported in your own shell — there's no overriding it from outside.
- `SYS_AUTOSTART` (the airframe selector) is a **persisted parameter**, not re-derived from the environment on every boot — it only auto-detects once, then a stale `parameters.bson` silently keeps reusing the old value regardless of what you change afterward.
- Adding a genuinely new model name would mean registering a new numbered airframe file (PX4 reserves `22000`–`22999` for exactly this) and then finding — which we never located — wherever `gz_<model>` Ninja targets actually get generated from that list, or force-writing `SYS_AUTOSTART` as a persisted parameter to bypass name-based detection entirely.

Rather than keep digging into PX4-internal plumbing indefinitely, we sidestepped it: `setup/install_sim_sensors.sh` patches our LiDAR + IMU sensors **directly into the locally-downloaded stock `x500` model**, so every standard PX4 command (`make px4_sitl gz_x500`, no overrides) just works, unmodified. The tradeoff: on a machine that's run this script, `x500` locally always means "the sensor-equipped variant." If you ever need the plain stock vehicle back, restore from the script's own backup (see the script for the path) or re-download via `simulation-gazebo --overwrite`.

The sensor definitions themselves still live at `simulation/models/x500_lidar/sensors.sdf.xml` in this repo — that's the source of truth (in version control, reviewable in PRs); the script just injects it into a file that lives outside the repo, under `~/.simulation-gazebo/`, per machine.

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

## WSL2 environment prerequisites (one-time, per machine)

Two real WSL2/Gazebo issues surfaced getting this running — fix both before touching PX4:

1. **Mirrored networking.** WSL2's default NAT networking breaks `gz-transport`'s peer discovery in some configurations. Add to `%USERPROFILE%\.wslconfig` on Windows (create it if it doesn't exist):
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
   Then from PowerShell: `wsl --shutdown`, and reopen your WSL2 terminal. Requires Windows 11 with a reasonably recent WSL version (`wsl --version`).
2. **Multicast route.** Even with mirrored networking, WSL2 doesn't always get a default multicast route, which `gz-transport` discovery also depends on:
   ```bash
   sudo ip route add 224.0.0.0/4 dev eth1   # adjust eth1 if your interface is named differently (`ip -4 addr`)
   ```
   This doesn't persist across reboots/WSL restarts — re-run it if Gazebo discovery ever mysteriously breaks again after a restart. (In our specific debugging session, the actual blocking bug turned out to be the startup race described below, not this — but this was a real, independently-confirmed gap worth fixing regardless.)

## The real blocker: a PX4/Gazebo startup race, not networking

In default (non-standalone) mode, PX4 spawns its own Gazebo process and then makes **one single** service call with a 1000ms timeout to spawn the vehicle — no retry. Under WSL2, Gazebo routinely takes longer than one second to finish initializing its services, so this one-shot call reliably times out (`Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.` — a misleading error message; the resource path is usually fine). The fix is `PX4_GZ_STANDALONE=1`, which switches PX4 to a retry-every-2-seconds loop instead — but that mode also means PX4 does **not** launch Gazebo itself; you launch it yourself first, in its own terminal, then point PX4 at it. See the exact sequence below.

## Build and run

```bash
# one-time: vendor the LIO backend + its own hard dependency
vcs import src < uav_localization.repos
rosdep install --from-paths src --ignore-src -r -y

# fast_lio pulls in ikd-Tree as a git submodule; vcs import doesn't init it
(cd src/fast_lio && git submodule update --init --recursive)

# livox_ros_driver2 ships two variant manifests instead of a plain package.xml
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml

# livox_ros_driver2 links against the Livox-SDK2 native library — build it
# once, system-wide (this is a plain CMake C++ library, not a ROS package):
#   git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
#   cd ~/Livox-SDK2 && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install

# livox_ros_driver2 needs a two-phase build plain `colcon build` can't do —
# use its own build script. It recognizes "jazzy" natively, no workaround needed.
source /opt/ros/jazzy/setup.bash
(cd src/livox_ros_driver2 && ./build.sh jazzy)
colcon build --symlink-install --packages-select fast_lio   # in case the submodule fix above landed after build.sh's own pass
source install/setup.bash

# one-time: download PX4's stock Gazebo models (Ctrl+C once the window opens —
# we only need the download, not this particular launch)
python3 ~/PX4-Autopilot/Tools/simulation/gz/simulation-gazebo

# one-time: patch our sensors into the downloaded x500 model
./setup/install_sim_sensors.sh

# terminal 1: Gazebo, standalone — leave running
cd ~/PX4-Autopilot
python3 Tools/simulation/gz/simulation-gazebo

# terminal 2: PX4 SITL, standalone mode (retries until it detects terminal 1's Gazebo)
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500

# terminal 3: bridge the simulated sensors into ROS 2
ros2 launch $(pwd)/simulation/launch/sensors_bridge.launch.xml

# terminal 4: in place of mock_pipeline.launch.xml
ros2 launch uav_bringup real_localization_pipeline.launch.xml
```

## Verifying this on your machine

Work through these in order — each one isolates a different layer, so if something's wrong you'll know which piece before moving to the next:

1. **PX4 reaches a flyable prompt.** After the terminal 1 + terminal 2 sequence above, terminal 2 should reach `pxh>` with `Startup script returned successfully`, and the Gazebo window should show the quad (the LiDAR cylinder may not be visually obvious under WSL2's software rendering — that's a rendering quirk, not proof the sensor is missing; verify via topics in the next step instead). If terminal 2 instead loops `Service call timed out as Gazebo has not been detected`, terminal 1 either isn't running or hasn't finished starting yet — wait longer before starting terminal 2.
2. **Sensors are actually publishing in Gazebo**, before touching ROS 2 at all:
   ```bash
   gz topic -l | grep -E 'lidar|imu'
   gz topic -e -t /world/default/model/x500_0/link/imu_link/sensor/imu/imu
   ```
   If these don't list/echo, `setup/install_sim_sensors.sh` either wasn't run or the patched model wasn't picked up — check `grep -c 'gps-denied-uav sensors' ~/.simulation-gazebo/models/x500/model.sdf` (expect `2`).
3. **The ROS 2 bridge is relaying it**, with `sensors_bridge.launch.xml` running:
   ```bash
   ros2 topic hz /lidar/points     # expect ~10 Hz, matching sensors.sdf.xml's lidar update_rate
   ros2 topic hz /imu/data         # expect ~200 Hz
   ros2 topic echo /imu/data --once
   ```
4. **FAST-LIO2 is producing odometry**, with the full `real_localization.launch.xml` running:
   ```bash
   ros2 topic hz /Odometry
   ros2 topic echo /Odometry --once
   ```
   If `/lidar/points` and `/imu/data` are both flowing but `/Odometry` never appears, the problem is inside the vendored FAST-LIO2 config — check its own log output first (`fast_lio_x500.yaml`'s topic names, `lidar_type`/`timestamp_unit` matching the simulated sensor). **This is the first unverified step** — everything before it is confirmed, this one hasn't been run yet.
5. **`lio_state_bridge` is relaying it correctly** into our contract:
   ```bash
   ros2 topic echo /localization/state
   ```
   Fly the vehicle a short distance (even a manual RC/QGroundControl takeoff) and confirm: `localization_ok: true`, `status: 0` (`STATUS_NOMINAL`), `pose.position` changing plausibly with movement, `header.frame_id: map`. Stop `/Odometry` (kill the FAST-LIO2 process) and confirm `/localization/state` transitions `NOMINAL -> DEGRADED -> LOST` over ~0.3s and ~1.5s respectively, matching the table above — that failure-path behavior matters as much as the happy path, since it's what Safety depends on.
6. **The full pipeline still holds together.** With `real_localization_pipeline.launch.xml` running end-to-end, `ros2 topic echo /safety/vehicle_command` should behave exactly like it did with `MockLocalization` in the Milestone 1 verification — `valid: true` once localization is healthy, `MODE_HOLD`/`valid: false` if you kill localization. If Safety's behavior differs at all between mock and real Localization, that's a contract violation somewhere in this chain, not a Safety bug — `LocalizationState` is supposed to be indistinguishable to downstream consumers regardless of producer.

Once (5) and (6) pass, this module has cleared the same bar `MockLocalization` already cleared — see [docs/TESTING.md](TESTING.md) for what "contract-tested" means and task 3 below for making that automatic rather than manual.

## Next tasks, roughly in order

1. Run verification steps 2-4 above — confirm FAST-LIO2 actually produces `/Odometry` from the real sensor topics. This is the first genuinely unverified step in the whole chain.
2. Timestamp synchronization between the LiDAR and IMU sources (`docs/CONVENTIONS.md` calls this out as your responsibility) — confirm the sim sensors are already synced or add correction.
3. Add contract tests for `lio_state_bridge` itself (currently only `MockLocalization`'s output is contract-tested) — same pattern as `tests/contract/test_node_contracts.py`, publishing synthetic `Odometry` and asserting the staleness/status table above, so verification step 5 above becomes a `pytest` assertion.
4. Improve the confidence heuristic past "is it fresh" — look at what the vendored backend actually exposes about registration quality (e.g. FAST-LIO2's ESKF covariance) instead of a fixed 0.9/0.3/0.0.
5. Tune `acc_cov`/`gyr_cov`/etc. in `fast_lio_x500.yaml` against the simulated IMU's actual noise, instead of FAST-LIO2's stock defaults.
6. Run the golden scenario (`docs/TESTING.md`) with real localization once the above lands, and compare drift against the mocked baseline.
