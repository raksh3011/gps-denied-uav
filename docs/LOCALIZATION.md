# Localization Module

Owner: Person 1. This documents the real Localization implementation, layered on top of the frozen `LocalizationState` contract in [docs/INTERFACES.md](INTERFACES.md). `MockLocalization` still exists and still runs — the rest of the team keeps developing against it; this doesn't replace it, it sits alongside it as an alternative producer of the same topic.

## Status

**Confirmed (2026-08-19/20, raksh's WSL2 machine): `simulation/worlds/x500_lidar.sdf` loads cleanly in `gz sim` alone (no PX4) and publishes both `lidar_link` and `imu_link` sensor topics, alongside the stock `base_link` sensors** — `gz topic -l` lists `/world/default/model/x500_lidar_0/link/lidar_link/sensor/lidar/scan/points` and `.../link/imu_link/sensor/imu/imu`. This took two real fixes to get right (see [Why the vehicle+sensors are a static world file](#why-the-vehiclesensors-are-a-static-world-file)): reverting a stale patch left on `x500_base/model.sdf` from an earlier abandoned approach, and explicitly declaring Gazebo's `Sensors`/`Imu`/`AirPressure`/`Contact` system plugins on the world — which turned out to normally be loaded dynamically by PX4's own `gz_bridge` module during its spawn call, not by Gazebo automatically, so removing PX4 from the loop (the whole point of this approach) had silently also removed the thing that made sensors work at all.

**Also confirmed: PX4 attaches to the running world via `PX4_GZ_MODEL_NAME=x500_lidar_0`** — logs `PX4_GZ_MODEL_NAME set, PX4 will attach to existing model`, reaches `pxh>` and `Ready for takeoff!`.

**Also confirmed: the ROS 2 bridge relays real data end-to-end** — `/imu/data` and `/lidar/points` both appear in `ros2 topic list`, and `ros2 topic hz /imu/data` measures ~195 Hz (matching the configured 200 Hz, low jitter). The full chain — Gazebo sensors -> `gz-transport` -> `ros_gz_bridge` -> ROS 2 topics — works.

One environment-level regression surfaced and was fixed along the way: **do not enable WSL2 mirrored networking for this project** — see [WSL2 environment prerequisites](#wsl2-environment-prerequisites-one-time-per-machine) below, it broke ROS 2 DDS discovery entirely.

`fastlio_mapping` also crashed on launch until `fast_lio_x500.yaml` was fixed to use the ROS 2 params-file structure (`/**: ros__parameters:`) instead of FAST-LIO2's own flat upstream YAML convention — see the comment at the top of that file. **Confirmed fixed: it no longer crashes**, reaches `Node init finished.`, and correctly subscribes to `/lidar/points`/`/imu/data` (verified via `ros2 node info /laser_mapping`) with matching QoS on both ends.

**Not yet confirmed: `fastlio_mapping` actually publishing `/Odometry`.** On the machine this was debugged on, running the full stack simultaneously — Gazebo (GUI + server), PX4, the ROS 2 bridge, `fastlio_mapping`, and `rviz2` — exceeded available resources (3.8GB RAM, load average >7, `/lidar/points` itself stalled since `gpu_lidar` needs active GPU rendering). Whether `fastlio_mapping` would actually produce odometry given uninterrupted sensor data is **still an open question**, not confirmed one way or the other. See [Running this on constrained hardware](#running-this-on-constrained-hardware) below before debugging further on a similarly limited machine.

What's still open, in order:

1. **Confirm FAST-LIO2 actually produces `/Odometry`**, on a machine with enough headroom to keep all sensor topics flowing continuously (headless Gazebo, no RViz — see below). If it still doesn't, next things to check: the point cloud's actual field layout (`ros2 topic echo /lidar/points --no-arr`) against what FAST-LIO2's `lidar_type: 2` parser expects — a mismatch there is a common real-world FAST-LIO2 setup issue and hasn't been ruled out.
2. `acc_cov`/`gyr_cov`/etc. in `fast_lio_x500.yaml` are FAST-LIO2's stock defaults, not tuned against the simulated IMU's actual noise characteristics.
3. `lio_state_bridge`'s confidence/status heuristic (see below) is a stand-in — timestamp-staleness-only, doesn't look at anything FAST-LIO2 exposes about registration quality/degeneracy.

## Why this design

Every other module (World Model, Planning, Safety) was built and tested against `MockLocalization` publishing on `/localization/state`. That contract doesn't change here — `lio_state_bridge` publishes the exact same `LocalizationState` message on the exact same topic. Swapping mock for real is a launch-file change (`mock_pipeline.launch.xml` -> `real_localization_pipeline.launch.xml`), not an interface change. See `docs/DEVELOPMENT.md` for the general "develop against mocks, swap in real later" workflow this is an instance of.

The actual state estimation is FAST-LIO2, not code we wrote. `lio_state_bridge` is deliberately thin: it does not do sensor fusion, it converts FAST-LIO2's `nav_msgs/Odometry` output into our contract and turns silence into an honest `localization_ok=false` instead of letting stale data look live. Keeping the adapter thin means swapping the backend later (a different LIO algorithm, or real hardware with a different driver) only touches this one file plus the launch file — not any downstream module.

## Why the vehicle+sensors are a static world file

Two earlier approaches were tried and abandoned before landing on the current one. Both are worth knowing about if this area breaks again.

**Attempt 1: a separate `x500_lidar` model name.** PX4's SITL launch plumbing — the `make px4_sitl gz_x500` Ninja target, and `PX4_SIM_MODEL`-based airframe auto-detection — turned out to be hardcoded to a small, fixed set of known model names, with real, confirmed reasons no override worked:

- `PX4_GZ_MODEL` (an env var we initially guessed at) isn't read anywhere in this PX4 version at all.
- The real variable is `PX4_SIM_MODEL` (format `gz_<model>`), but the `gz_x500` Ninja target hardcodes `PX4_SIM_MODEL=gz_x500` as an inline command-line assignment, which always wins over anything exported in your own shell.
- `SYS_AUTOSTART` (the airframe selector) is a **persisted parameter**, not re-derived from the environment on every boot — a stale `parameters.bson` silently keeps reusing the old value regardless of what you change afterward.
- A genuinely new model name would need a new numbered airframe file (PX4 reserves `22000`–`22999` for this) plus finding — never located — wherever `gz_<model>` Ninja targets actually get generated.

**Attempt 2: patch our sensors directly into the downloaded `x500_base` model**, so PX4's existing `x500` spawn (dynamically assembled from `x500_base` + motor plugins via a runtime `create` service call — confirmed via Gazebo's own SDF-parsing warnings, which show `x500` as `<data-string>` rather than a file path, while `x500_base`'s warnings show a real on-disk path) would pick them up unmodified. This got as far as `<include merge='true'>` genuinely detecting and merging our patched content (confirmed via a real "duplicate link name" error when an older stale patch was also present), but multiple clean, verified relaunches still didn't produce sensor topics with only one correctly-applied copy of the patch, and we couldn't pin down why before the debugging cycle itself became the problem: **every single test required a full PX4 launch just to find out whether a sensor existed**, which is far too slow a loop for iterating on SDF content.

**Current approach: a fully static, self-contained world file** (`simulation/worlds/x500_lidar.sdf`) — the vehicle (`x500_base` + PX4's motor plugins, copied from the downloaded `x500/model.sdf` + our LiDAR/IMU) is placed directly in the world via a plain `<include>`, the same standard, well-tested SDF mechanism, but resolved once at **world load time** instead of PX4's dynamic runtime spawn path. This can be verified with `gz sim` alone — load the world, check `gz topic -l` — with no PX4 involved at all, which is the property Attempt 2 was missing. PX4 then **attaches** to the already-placed vehicle via `PX4_GZ_MODEL_NAME=x500_lidar_0` (a separate, simpler branch in `ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator` that subscribes to an existing model instead of calling the `create` service), rather than spawning anything itself.

The full vehicle+sensor definition lives at `simulation/worlds/x500_lidar.sdf`, version-controlled in this repo — nothing outside the repo needs patching anymore.

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

**Do NOT enable WSL2 mirrored networking mode (`networkingMode=mirrored` in `.wslconfig`) for this project.** We tried it — it was originally suspected as the fix for a `gz-transport` discovery issue that turned out to actually be the PX4/Gazebo startup race described below, unrelated to networking. Mirrored mode routes WSL2 traffic through Windows' real network stack, including Windows Defender Firewall, which **completely broke ROS 2 DDS discovery** (`ros2 topic list` hung indefinitely, even with nothing else running, even after a full `wsl --shutdown`) — a much worse problem than the one it was meant to solve. If a machine has mirrored mode enabled from following an older version of this doc, revert it: remove `networkingMode=mirrored` from `%USERPROFILE%\.wslconfig`, then `wsl --shutdown` from PowerShell and reopen your terminal. Plain default (NAT) networking is correct for this stack.

If Gazebo's own peer discovery (`gz-transport`, separate from ROS 2's DDS) ever seems broken independent of the above, a missing multicast route is a real, independently-confirmed gap worth checking:
```bash
ip route show | grep 224
sudo ip route add 224.0.0.0/4 dev eth1   # if missing; adjust eth1 if your interface is named differently (`ip -4 addr`)
```
This doesn't persist across reboots/WSL restarts. In our debugging session this was never actually the blocking issue (the real one was the startup race below), but it's cheap to check.

## The PX4/Gazebo startup race (confirmed, applies regardless of which vehicle approach)

In default (non-standalone) mode, PX4 spawns its own Gazebo process and then makes **one single** service call with a 1000ms timeout to spawn the vehicle — no retry. Under WSL2, Gazebo routinely takes longer than one second to finish initializing its services, so this one-shot call reliably times out (`Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.` — a misleading error message; the resource path is usually fine). The fix is `PX4_GZ_STANDALONE=1`, which switches PX4 to a retry-every-2-seconds loop instead — but that mode also means PX4 does **not** launch Gazebo itself; you launch it yourself first, in its own terminal, then point PX4 at it.

This was confirmed for the dynamic `create`-service spawn path. The `PX4_GZ_MODEL_NAME` attach path (what we use now) subscribes to an existing model's topics rather than making a timed RPC call, so it may not have the same one-shot race at all — but launch Gazebo first regardless, and keep `PX4_GZ_STANDALONE=1` set; there's no reason to believe it hurts, and it hasn't been specifically tested without it.

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

# one-time: download PX4's stock Gazebo models, so model://x500_base
# resolves (Ctrl+C once the window opens — we only need the download)
python3 ~/PX4-Autopilot/Tools/simulation/gz/simulation-gazebo

# terminal 1: Gazebo, loading OUR world file directly (not simulation-gazebo's
# own launcher — we don't want its dynamic x500 spawn, our vehicle is
# already placed statically in this world file). Add -s (headless, no GUI)
# instead of just -r if you're on constrained hardware — see below.
export GZ_SIM_RESOURCE_PATH=~/.simulation-gazebo/models
cd ~/gps-denied-uav
gz sim -r simulation/worlds/x500_lidar.sdf

# terminal 2: PX4 SITL, attaching to the already-placed vehicle instead of spawning one
cd ~/PX4-Autopilot
PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=x500_lidar_0 make px4_sitl gz_x500

# terminal 3: bridge the simulated sensors into ROS 2
cd ~/gps-denied-uav
ros2 launch $(pwd)/simulation/launch/sensors_bridge.launch.xml

# terminal 4: in place of mock_pipeline.launch.xml
ros2 launch uav_bringup real_localization_pipeline.launch.xml
```

## Running this on constrained hardware

The full stack — Gazebo (GUI + physics + rendering), PX4 SITL, the ROS 2 bridge, `fastlio_mapping`, and `rviz2` — is genuinely heavy. On a 3.8GB-RAM WSL2 VM this reliably overloaded (load average >7, `/lidar/points` stalled since `gpu_lidar` needs active GPU rendering to produce anything) once `fastlio_mapping` and `rviz2` joined the already-running Gazebo+PX4+bridge. If you hit the same thing:

- **Run Gazebo headless**: `gz sim -s -r simulation/worlds/x500_lidar.sdf` (`-s` = server only, no 3D GUI window). Sensor topics publish identically either way — the GUI is purely visual and was the single heaviest process.
- **Skip `rviz2`.** `real_localization.launch.xml` includes it via FAST-LIO2's own `mapping.launch.py` (not something we launch directly) purely for visualization; it isn't needed to verify `/Odometry` is being published. If it can't be disabled via a launch argument, just `pkill -9 -f rviz2` right after the launch starts — everything else keeps running.
- Watch `free -h` and `top` between steps. If `/lidar/points`/`/imu/data` stop publishing partway through a session that worked a moment ago, that's this — not a config regression — check load before re-debugging config.

## Verifying this on your machine

Work through these in order — each one isolates a different layer, so if something's wrong you'll know which piece before moving to the next. **Step 1 needs no PX4 at all** — do it first, in isolation, before touching PX4.

1. **The world loads and sensors publish, with `gz sim` alone** (terminal 1 above, nothing else running):
   ```bash
   gz topic -l | grep -E 'lidar|imu'
   gz topic -e -t /world/default/model/x500_lidar_0/link/imu_link/sensor/imu/imu -n 1
   ```
   Expect three topics: the original stock `imu_sensor` on `base_link`, plus our new `lidar_link` and `imu_link` ones. If ours are missing, the problem is entirely inside `simulation/worlds/x500_lidar.sdf` — fix it here before going anywhere near PX4. Check `gz sim`'s own terminal output for SDF parsing errors/warnings first.
2. **PX4 attaches successfully.** With terminal 1 still running, start terminal 2. It should reach `pxh>` with `Startup script returned successfully` and log `PX4_GZ_MODEL_NAME set, PX4 will attach to existing model`. If it doesn't find the model, double check the name matches exactly (`x500_lidar_0`, matching both the world file's `<model name="x500_lidar_0">` and the `PX4_GZ_MODEL_NAME` value).
3. **The ROS 2 bridge is relaying it**, with `sensors_bridge.launch.xml` running:
   ```bash
   ros2 topic hz /lidar/points     # expect ~10 Hz, matching x500_lidar.sdf's lidar update_rate
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
   Fly the vehicle a short distance (even a manual RC/QGroundControl takeoff) and confirm: `localization_ok: true`, `status: 0` (`STATUS_NOMINAL`), `pose.position` changing plausibly with movement, `header.frame_id: map`. Stop `/Odometry` (kill the FAST-LIO2 process) and confirm `/localization/state` transitions `NOMINAL -> DEGRADED -> LOST` over ~0.3s and ~1.5s respectively, matching the table above — that failure-path behavior matters as much as the happy path, since it's what Safety depends on.
6. **The full pipeline still holds together.** With `real_localization_pipeline.launch.xml` running end-to-end, `ros2 topic echo /safety/vehicle_command` should behave exactly like it did with `MockLocalization` in the Milestone 1 verification — `valid: true` once localization is healthy, `MODE_HOLD`/`valid: false` if you kill localization. If Safety's behavior differs at all between mock and real Localization, that's a contract violation somewhere in this chain, not a Safety bug — `LocalizationState` is supposed to be indistinguishable to downstream consumers regardless of producer.

Once (5) and (6) pass, this module has cleared the same bar `MockLocalization` already cleared — see [docs/TESTING.md](TESTING.md) for what "contract-tested" means and task 3 below for making that automatic rather than manual.

7. **See the whole real stack fly, in RViz, against the real Gazebo world**: `ros2 launch uav_bringup full_real_pipeline.launch.xml` instead of `real_localization_pipeline.launch.xml` — same Gazebo/PX4/bridge prerequisites, but this one also runs the real World Model (`real_world_model`, clustering FAST-LIO2's own `/cloud_registered` into a live occupancy map and obstacles — see [docs/WORLD_MODEL.md](WORLD_MODEL.md)) and real Planning (`real_planner`, Theta* + D* Lite + Margasoochi — see [docs/PLANNING.md](PLANNING.md)) instead of their mocks, plus `viz_bridge`/`rviz2` for a live view: the vehicle's real pose, its real flown trail, the real occupied-voxel map building up as it flies, and the real planned path. This is the RViz counterpart to `demo_mission.launch.xml`, which exists only because the primary dev machine can't run Gazebo — on capable hardware, this is the one to use.

## Next tasks, roughly in order

1. **Confirm FAST-LIO2 produces `/Odometry`**, on hardware with enough headroom to keep `/lidar/points`/`/imu/data` flowing continuously (headless Gazebo + no `rviz2`, see [Running this on constrained hardware](#running-this-on-constrained-hardware)). Steps 1-3 (world loads standalone, PX4 attaches, ROS 2 bridge relays real data) are all confirmed — this is the only unverified link left in the sensor->odometry chain, and it's unverified because of resource limits on the machine it was attempted on, not because of a known bug.
2. If `/Odometry` still doesn't appear on capable hardware: check the point cloud's actual field layout (`ros2 topic echo /lidar/points --no-arr`) against what FAST-LIO2's `lidar_type: 2` parser expects — a mismatch there is a common real-world FAST-LIO2 setup issue and hasn't been ruled out yet.
3. Timestamp synchronization between the LiDAR and IMU sources (`docs/CONVENTIONS.md` calls this out as your responsibility) — confirm the sim sensors are already synced or add correction.
4. Add contract tests for `lio_state_bridge` itself (currently only `MockLocalization`'s output is contract-tested) — same pattern as `tests/contract/test_node_contracts.py`, publishing synthetic `Odometry` and asserting the staleness/status table above, so verification step 5 above becomes a `pytest` assertion.
5. Improve the confidence heuristic past "is it fresh" — look at what the vendored backend actually exposes about registration quality (e.g. FAST-LIO2's ESKF covariance) instead of a fixed 0.9/0.3/0.0.
6. Tune `acc_cov`/`gyr_cov`/etc. in `fast_lio_x500.yaml` against the simulated IMU's actual noise, instead of FAST-LIO2's stock defaults.
7. Run the golden scenario (`docs/TESTING.md`) with real localization once the above lands, and compare drift against the mocked baseline.
