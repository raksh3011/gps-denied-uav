// Copyright 2026 UAV Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// SafetyMonitor: the real Safety Supervisor decision logic. ROS-free (same
// pattern as uav_planning_core/uav_world_model_core) so it's unit-testable
// without a ROS runtime — real_safety_node is the only place that converts
// our topics into calls on this class.
//
// Design, per docs/SAFETY.md:
// - Localization DEGRADED is NOT disqualifying on its own: Planning's own
//   confidence-adaptive risk margin (CARM, see docs/PLANNING.md) already
//   widens obstacle standoff and the local planner already reacts to it.
//   Safety defers to that and only downgrades reported health to WARN.
//   Safety only actively intervenes (HOLD/LAND) when localization is
//   fully LOST, or when trajectory/map data is missing, stale, or
//   invalid — situations CARM cannot compensate for because there is
//   nothing trustworthy left to plan against.
// - A momentary fault (one stale sample, one bad tick) gets `valid=false`
//   — per the VehicleCommand contract, that means "reject and hold the
//   last safe command," which is already a safe default and avoids
//   spamming mode changes for a single glitch.
// - A SUSTAINED loss of localization (LOST or invalid for longer than
//   lost_hold_timeout_s) escalates to an explicit, authoritative
//   `MODE_LAND` command (`valid=true`) — holding forever on a position
//   estimate that's known to be gone isn't actually safe; landing in
//   place is. RTL is not used here: RTL needs a trustworthy position to
//   navigate home, which is exactly what's missing.
// - An independent obstacle-clearance check on the trajectory's near-term
//   points, against the live ObstacleSet, is Safety's own defense in
//   depth — it does not trust that Planning's own avoidance is bug-free;
//   if the path Planning claims is `valid` actually intersects a
//   currently-tracked obstacle, Safety overrides with HOLD regardless.
#ifndef UAV_SAFETY__SAFETY_MONITOR_HPP_
#define UAV_SAFETY__SAFETY_MONITOR_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace uav_safety
{

struct ObstacleReading
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

// Plain snapshot of everything Safety consumes, gathered by
// real_safety_node from the live topics each tick. Ages are seconds since
// the node's own wall clock last received that message (infinity if
// never received) — NOT derived from header.stamp, since that would
// conflate simulation-time jumps with real staleness.
struct SafetyInputs
{
  bool loc_present{false};
  bool loc_ok{false};
  uint8_t loc_status{0};   // LocalizationState::STATUS_*
  double loc_age_s{1e9};

  bool traj_present{false};
  bool traj_valid{false};
  double traj_age_s{1e9};
  Eigen::Vector3d traj_target_position{Eigen::Vector3d::Zero()};
  float traj_target_yaw{0.0F};
  // First few points of the trajectory, for the independent obstacle
  // clearance check — not the whole path, so the check stays cheap.
  std::vector<Eigen::Vector3d> traj_near_points;

  bool map_present{false};
  bool map_valid{false};
  double map_age_s{1e9};

  bool planner_present{false};
  uint8_t planner_state{0};   // PlannerStatus::STATE_*

  std::vector<ObstacleReading> obstacles;
};

// Mirrors VehicleCommand + SystemHealth's fields exactly (plain types, no
// ROS message dependency in this ROS-free core).
struct SafetyOutputs
{
  uint8_t cmd_mode{0};
  Eigen::Vector3d cmd_position{Eigen::Vector3d::Zero()};
  float cmd_yaw{0.0F};
  bool cmd_valid{false};

  uint8_t overall_level{0};
  uint8_t localization_level{0};
  uint8_t world_model_level{0};
  uint8_t planning_level{0};
  uint8_t vehicle_link_level{0};   // no feedback topic yet, always OK — see docs/SAFETY.md
  std::vector<std::string> active_faults;
};

class SafetyMonitor
{
public:
  struct Params
  {
    double staleness_timeout_s{0.5};       // loc/trajectory
    double map_staleness_timeout_s{1.0};   // matches LocalMap contract's own >1s guidance
    double lost_hold_timeout_s{3.0};       // sustained-loss duration before HOLD escalates to LAND
    double min_obstacle_clearance_m{0.3};  // independent redundant check margin
  };

  // Two overloads rather than a `Params params = Params{}` default
  // argument: some GCC versions reject a nested struct's own default
  // member initializers as a default-argument value inside the SAME
  // enclosing class body (rejected even though Params is itself already
  // complete at that point) — delegating from a no-arg constructor in
  // the .cpp sidesteps it entirely.
  SafetyMonitor();
  explicit SafetyMonitor(Params params);

  // dt_s: wall time elapsed since the previous update() call — used to
  // advance the sustained-loss timer. Call once per tick.
  SafetyOutputs update(const SafetyInputs & in, double dt_s);

private:
  Params params_;
  double lost_duration_s_{0.0};
};

}  // namespace uav_safety

#endif  // UAV_SAFETY__SAFETY_MONITOR_HPP_
