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

#include "uav_safety/safety_monitor.hpp"

#include <algorithm>

namespace uav_safety
{

namespace
{
// Mirror the frozen enum values from uav_interfaces (kept as plain ints
// here, same convention DStarLitePlanner::riskBandFor uses, so this core
// stays message-type-agnostic).
constexpr uint8_t kLocStatusDegraded = 1;   // LocalizationState::STATUS_DEGRADED
constexpr uint8_t kLocStatusLost = 2;       // LocalizationState::STATUS_LOST

constexpr uint8_t kPlannerFailed = 5;   // PlannerStatus::STATE_FAILED

constexpr uint8_t kModePosition = 0;   // VehicleCommand::MODE_POSITION
constexpr uint8_t kModeLand = 2;       // VehicleCommand::MODE_LAND
constexpr uint8_t kModeHold = 4;       // VehicleCommand::MODE_HOLD

constexpr uint8_t kLevelOk = 0;
constexpr uint8_t kLevelWarn = 1;
constexpr uint8_t kLevelCritical = 2;
constexpr uint8_t kLevelFailsafe = 3;
}  // namespace

SafetyMonitor::SafetyMonitor()
: SafetyMonitor(Params())
{
}

SafetyMonitor::SafetyMonitor(Params params)
: params_(params)
{
}

SafetyOutputs SafetyMonitor::update(const SafetyInputs & in, double dt_s)
{
  SafetyOutputs out;

  // --- Localization ---
  const bool loc_stale = !in.loc_present || in.loc_age_s > params_.staleness_timeout_s;
  const bool loc_lost = in.loc_present && (!in.loc_ok || in.loc_status == kLocStatusLost);
  const bool loc_degraded = in.loc_present && in.loc_ok && !loc_stale &&
    in.loc_status == kLocStatusDegraded;
  const bool loc_bad = loc_stale || loc_lost;

  if (loc_bad) {
    out.localization_level = kLevelCritical;
  } else if (loc_degraded) {
    out.localization_level = kLevelWarn;
  } else {
    out.localization_level = kLevelOk;
  }

  // --- Trajectory ---
  const bool traj_stale = !in.traj_present || in.traj_age_s > params_.staleness_timeout_s;
  const bool traj_bad = traj_stale || !in.traj_valid;

  // --- World Model / map ---
  const bool map_stale = in.map_present && in.map_age_s > params_.map_staleness_timeout_s;
  const bool map_bad = !in.map_present || !in.map_valid;
  if (map_bad) {
    out.world_model_level = kLevelCritical;
  } else if (map_stale) {
    out.world_model_level = kLevelWarn;
  } else {
    out.world_model_level = kLevelOk;
  }

  // --- Planning ---
  const bool planner_failed = in.planner_present && in.planner_state == kPlannerFailed;
  if (traj_bad || planner_failed) {
    out.planning_level = kLevelCritical;
  } else if (!in.planner_present) {
    out.planning_level = kLevelWarn;
  } else {
    out.planning_level = kLevelOk;
  }

  // --- Independent obstacle-clearance check (defense in depth) ---
  // Doesn't trust Planning's own `valid` flag — checks the trajectory's
  // near-term points against the live ObstacleSet directly.
  bool obstacle_violation = false;
  if (!traj_bad) {
    for (const auto & point : in.traj_near_points) {
      for (const auto & obstacle : in.obstacles) {
        const double clearance = (point - obstacle.position).norm() - obstacle.radius;
        if (clearance < params_.min_obstacle_clearance_m) {
          obstacle_violation = true;
          break;
        }
      }
      if (obstacle_violation) {break;}
    }
  }

  // --- Sustained-loss timer ---
  if (loc_bad) {
    lost_duration_s_ += dt_s;
  } else {
    lost_duration_s_ = 0.0;
  }
  const bool sustained_loss = loc_bad && lost_duration_s_ > params_.lost_hold_timeout_s;

  // --- Command decision ---
  out.active_faults.clear();
  if (loc_stale) {out.active_faults.push_back("LOC_STALE");}
  if (in.loc_present && !in.loc_ok) {out.active_faults.push_back("LOC_NOT_OK");}
  if (in.loc_present && in.loc_status == kLocStatusLost) {out.active_faults.push_back("LOC_LOST");}
  if (loc_degraded) {out.active_faults.push_back("LOC_DEGRADED");}
  if (traj_stale) {out.active_faults.push_back("TRAJ_STALE");}
  if (in.traj_present && !in.traj_valid) {out.active_faults.push_back("TRAJ_INVALID");}
  if (map_bad) {out.active_faults.push_back("MAP_INVALID");}
  if (map_stale) {out.active_faults.push_back("MAP_STALE");}
  if (planner_failed) {out.active_faults.push_back("PLANNER_FAILED");}
  if (obstacle_violation) {out.active_faults.push_back("OBSTACLE_CLEARANCE_VIOLATION");}

  if (sustained_loss) {
    // Holding forever on a position estimate known to be gone isn't
    // actually safe — land in place. RTL is not used: it needs a
    // trustworthy position to navigate home, which is exactly what's
    // missing here.
    out.active_faults.push_back("SUSTAINED_LOSS_LANDING");
    out.cmd_mode = kModeLand;
    out.cmd_valid = true;
    out.overall_level = kLevelFailsafe;
  } else if (loc_bad || traj_bad || obstacle_violation) {
    // A momentary fault: per the VehicleCommand contract, valid=false
    // means the consumer rejects this command and holds the last safe
    // one — already a safe default, no need for an authoritative mode
    // change over a single glitch.
    out.cmd_mode = kModeHold;
    out.cmd_valid = false;
    out.overall_level = kLevelCritical;
  } else {
    // Healthy, including loc_degraded: CARM (Planning's own confidence-
    // adaptive risk margin) already responded to degraded localization
    // by widening obstacle standoff — Safety defers to that and only
    // reflects it in reported health, not by blocking the command.
    out.cmd_mode = kModePosition;
    out.cmd_position = in.traj_target_position;
    out.cmd_yaw = in.traj_target_yaw;
    out.cmd_valid = true;
    out.overall_level = std::max(
      {out.localization_level, out.world_model_level, out.planning_level,
        out.vehicle_link_level});
  }

  return out;
}

}  // namespace uav_safety
