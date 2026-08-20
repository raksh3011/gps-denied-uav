# Vehicle / PX4 Interface Module

Owner: Person 4. Real `VehicleCommand` -> PX4 setpoint/command translation, per the frozen contract in [docs/INTERFACES.md](INTERFACES.md). `MockVehicle` still exists and still runs — swapping in `real_vehicle` is a launch-file change once `px4_msgs` is vendored, matching the workflow in [docs/DEVELOPMENT.md](DEVELOPMENT.md).

## Status

**Different from every other "real" module in this repo — read this before assuming anything here works.** World Model, Planning, and Safety were all built, unit-tested, contract-tested, and confirmed passing on a real machine before being called done. This module's core logic (`Px4CommandBridge`) has that same treatment — 15 gtests, zero ROS/PX4 dependency, fully confirmed. But the actual PX4 wiring (`real_vehicle_node`, `uav_vehicle.repos`) has **not been built or run against real PX4 even once**. It can't be, on the machine this was written on — it needs `px4_msgs` vendored and an actual PX4 SITL instance to talk to, exactly the setup Localization's `/Odometry` verification is still waiting on.

- `uav_vehicle_core` (C++ library, no ROS or px4_msgs dependency): `Px4CommandBridge` — ENU->NED frame conversion, one-shot vs. streamed command translation, an arm/offboard-switch warm-up state machine. 15 gtests, all confirmed passing.
- `real_vehicle_node` (ROS 2 node): only compiles if `px4_msgs` is present in the workspace (`find_package(px4_msgs QUIET)` in `CMakeLists.txt` — building `uav_vehicle` without it is unaffected; `mock_vehicle` and the core library build and test regardless). **Never compiled** — no `px4_msgs` install was available to compile against here.

Known, specific risk areas (not hypothetical — these are the actual places PX4's ROS 2 message API has changed across releases):
1. **Topic names.** `/fmu/out/vehicle_status` has been renamed `vehicle_status_v1` in some PX4/px4_msgs release combinations. Exposed as a ROS parameter (`vehicle_status_topic`) specifically so this can be corrected without a rebuild — but the default is a guess, not a confirmed value for `PX4_FIRMWARE_TAG=v1.15.0` (`setup/versions.txt`).
2. **`VehicleStatus` enum values.** `arming_state`/`nav_state` are read via px4_msgs' own symbolic constants (`VehicleStatus::ARMING_STATE_ARMED`, `VehicleStatus::NAVIGATION_STATE_OFFBOARD`) rather than hardcoded numbers specifically because the numeric values have changed across releases — the symbols should resolve correctly for whatever version actually gets vendored, but this has never been compiled to confirm the symbols themselves still exist under those exact names in `release/1.15`.
3. **px4_msgs branch/version.** `uav_vehicle.repos` pins `release/1.15` to match `PX4_FIRMWARE_TAG=v1.15.0` — if the PX4-Autopilot checkout has drifted, this must be updated too, or uORB field mismatches over µXRCE-DDS can silently corrupt data rather than fail loudly.

MAVLink command IDs used for arm/disarm (400), mode-switch (176), land (21), and RTL (20) are stable across the whole MAVLink/PX4 ecosystem and are the one part of this not flagged as at-risk.

## Design

```
/safety/vehicle_command (ENU) ──> real_vehicle_node ──> Px4CommandBridge ──┬──> /fmu/in/offboard_control_mode
                                        ^                                  ├──> /fmu/in/trajectory_setpoint  (NED)
                                        │                                  └──> /fmu/in/vehicle_command (arm/mode/land/RTL)
                          /fmu/out/vehicle_status (armed, offboard_active)
```

Decisions that matter, all inside the ROS-free, fully-tested `Px4CommandBridge`:

- **`valid=false` means keep streaming the last known-good setpoint, not go silent.** Per the `VehicleCommand` contract, an invalid command means "reject and hold" — but *stopping* the setpoint stream entirely would starve PX4's own offboard-timeout watchdog and hand control to PX4's internal failsafe instead of leaving the decision with our own Safety module. So the bridge keeps re-publishing the last accepted setpoint until a new valid command arrives.
- **`MODE_HOLD` freezes a *position*, not a velocity=0 setpoint.** A held position setpoint lets PX4 actively correct drift back toward that point; a zero-velocity setpoint does not resist drift at all. The position is captured once, the first tick HOLD begins, not re-read from the command every tick (which might not carry a meaningful position in HOLD mode anyway).
- **LAND/RTL/DISARM are one-shot**, tracked with an "already issued" flag reset only when the mode changes away and back — PX4's own `VehicleCommand` uORB message is a discrete command, not something meant to be re-sent 20 times a second like a setpoint.
- **Arm and the offboard-mode switch are gated behind a warm-up counter** (`offboard_warmup_ticks`, default 10, matching PX4's own ROS 2 `offboard_control_cpp` example) — PX4 refuses an offboard-mode switch request until it's seen a live setpoint stream for a bit, so the bridge counts consecutive streaming ticks before requesting the switch, then requests arm only after PX4 confirms offboard is actually active. Warm-up never accumulates during LAND/RTL/DISARM, which are deliberately leaving offboard control.

## Parameters (all on `real_vehicle`, once buildable)

| Parameter | Default | Meaning |
|---|---|---|
| `offboard_warmup_ticks` | 10 | streaming ticks before requesting the offboard-mode switch |
| `rate_hz` | 20.0 | publish rate (PX4 requires >=2Hz for the offboard setpoint stream) |
| `offboard_control_mode_topic` | `/fmu/in/offboard_control_mode` | **verify against your vendored px4_msgs/PX4** |
| `trajectory_setpoint_topic` | `/fmu/in/trajectory_setpoint` | **verify against your vendored px4_msgs/PX4** |
| `vehicle_command_topic` | `/fmu/in/vehicle_command` | **verify against your vendored px4_msgs/PX4** |
| `vehicle_status_topic` | `/fmu/out/vehicle_status` | **verify — known to be renamed `vehicle_status_v1` in some releases** |

## How to verify

The core, right now, no PX4/px4_msgs needed:

```bash
cd ~/gps-denied-uav && colcon build --packages-select uav_vehicle && colcon test --packages-select uav_vehicle && colcon test-result --verbose
```

The real node, once px4_msgs is vendored and PX4 SITL is running (see [docs/LOCALIZATION.md](LOCALIZATION.md) for the confirmed Gazebo+PX4 startup sequence — this module reuses it, nothing PX4-side is different):

```bash
vcs import src < uav_vehicle.repos
colcon build --packages-select px4_msgs
colcon build --packages-select uav_vehicle   # now picks up real_vehicle too
```

```bash
ros2 topic hz /fmu/out/vehicle_status   # confirm the topic name before anything else
ros2 run uav_vehicle real_vehicle
ros2 topic echo /fmu/in/trajectory_setpoint
```

Watch PX4's own console for the arming/offboard-switch sequence actually taking effect — that's the real confirmation this module needs and doesn't have yet.

## Next tasks, roughly in order

1. **Vendor `px4_msgs` and get `real_vehicle_node` to compile at all** — the first genuinely unverified step; field/constant names may not match exactly what's assumed above.
2. **Confirm the four topic names** against the actual running PX4 instance, correcting the ROS parameter defaults here and in any launch file once known.
3. **Confirm the arm -> offboard-switch -> setpoint-tracking sequence** actually flies the vehicle in Gazebo SITL, end to end.
4. **Feed `VehicleStatus` back into Safety's `vehicle_link_level`** — the frozen `SystemHealth` contract has a field for this but no producer yet (documented as a known gap in [docs/SAFETY.md](SAFETY.md)); once this module has a real link to PX4, closing that gap is a natural, contract-compatible follow-up (a `.msg`-adding PR, not a breaking change).
