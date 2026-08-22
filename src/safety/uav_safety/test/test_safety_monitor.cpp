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

#include <gtest/gtest.h>

#include <algorithm>

#include "uav_safety/safety_monitor.hpp"

using uav_safety::ObstacleReading;
using uav_safety::SafetyInputs;
using uav_safety::SafetyMonitor;

namespace
{
constexpr uint8_t kStatusNominal = 0;
constexpr uint8_t kStatusDegraded = 1;
constexpr uint8_t kStatusLost = 2;

constexpr uint8_t kModePosition = 0;
constexpr uint8_t kModeLand = 2;
constexpr uint8_t kModeHold = 4;

constexpr uint8_t kLevelOk = 0;
constexpr uint8_t kLevelWarn = 1;
constexpr uint8_t kLevelCritical = 2;
constexpr uint8_t kLevelFailsafe = 3;

constexpr uint8_t kPlannerExecuting = 2;
constexpr uint8_t kPlannerFailed = 5;

SafetyInputs healthyInputs()
{
  SafetyInputs in;
  in.loc_present = true;
  in.loc_ok = true;
  in.loc_status = kStatusNominal;
  in.loc_age_s = 0.05;

  in.traj_present = true;
  in.traj_valid = true;
  in.traj_age_s = 0.05;
  in.traj_target_position = Eigen::Vector3d(5.0, 0.0, 1.0);
  in.traj_target_yaw = 0.0F;
  in.traj_near_points = {Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d(2.0, 0.0, 1.0)};

  in.map_present = true;
  in.map_valid = true;
  in.map_age_s = 0.1;

  in.planner_present = true;
  in.planner_state = kPlannerExecuting;
  return in;
}
}  // namespace

TEST(SafetyMonitor, ForwardsValidCommandWhenAllHealthy) {
  SafetyMonitor monitor;
  const auto out = monitor.update(healthyInputs(), 0.05);

  EXPECT_TRUE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModePosition);
  EXPECT_EQ(out.cmd_position, Eigen::Vector3d(5.0, 0.0, 1.0));
  EXPECT_EQ(out.overall_level, kLevelOk);
  EXPECT_TRUE(out.active_faults.empty());
}

TEST(SafetyMonitor, DegradedLocalizationDoesNotBlockCommand) {
  // Margasoochi already responds to degraded localization on the Planning side;
  // Safety must defer, not double-block — see safety_monitor.hpp's
  // design comment.
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.loc_status = kStatusDegraded;

  const auto out = monitor.update(in, 0.05);
  EXPECT_TRUE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModePosition);
  EXPECT_EQ(out.localization_level, kLevelWarn);
  EXPECT_EQ(out.overall_level, kLevelWarn);
}

TEST(SafetyMonitor, StaleLocalizationHoldsWithInvalidCommand) {
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.loc_age_s = 5.0;   // way past staleness_timeout_s default (0.5s)

  const auto out = monitor.update(in, 0.05);
  EXPECT_FALSE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModeHold);
  EXPECT_EQ(out.overall_level, kLevelCritical);
}

TEST(SafetyMonitor, InvalidTrajectoryHolds) {
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.traj_valid = false;

  const auto out = monitor.update(in, 0.05);
  EXPECT_FALSE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModeHold);
}

TEST(SafetyMonitor, PlannerFailedIsReflectedAsCritical) {
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.planner_state = kPlannerFailed;

  const auto out = monitor.update(in, 0.05);
  EXPECT_EQ(out.planning_level, kLevelCritical);
  const auto & faults = out.active_faults;
  EXPECT_NE(std::find(faults.begin(), faults.end(), "PLANNER_FAILED"), faults.end());
}

TEST(SafetyMonitor, InvalidMapIsCriticalWorldModelLevel) {
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.map_valid = false;

  const auto out = monitor.update(in, 0.05);
  EXPECT_EQ(out.world_model_level, kLevelCritical);
}

TEST(SafetyMonitor, ObstacleOnTrajectoryOverridesValidFlagWithHold) {
  // Independent defense-in-depth check: even though Planning claims the
  // trajectory is valid, an obstacle sitting on a near-term point must
  // still force a HOLD.
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.obstacles = {ObstacleReading{Eigen::Vector3d(1.0, 0.0, 1.0), 0.5}};   // right on a near point

  const auto out = monitor.update(in, 0.05);
  EXPECT_FALSE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModeHold);
  const auto & faults = out.active_faults;
  EXPECT_NE(
    std::find(faults.begin(), faults.end(), "OBSTACLE_CLEARANCE_VIOLATION"), faults.end());
}

TEST(SafetyMonitor, FarObstacleDoesNotTriggerViolation) {
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.obstacles = {ObstacleReading{Eigen::Vector3d(50.0, 50.0, 50.0), 0.5}};

  const auto out = monitor.update(in, 0.05);
  EXPECT_TRUE(out.cmd_valid);
}

TEST(SafetyMonitor, TransientLossHoldsNotLands) {
  SafetyMonitor monitor;
  auto in = healthyInputs();
  in.loc_status = kStatusLost;
  in.loc_ok = false;

  const auto out = monitor.update(in, 0.1);   // one tick, well under lost_hold_timeout_s
  EXPECT_FALSE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModeHold);
  EXPECT_EQ(out.overall_level, kLevelCritical);
}

TEST(SafetyMonitor, SustainedLossEscalatesToLand) {
  SafetyMonitor::Params params;
  params.lost_hold_timeout_s = 1.0;
  SafetyMonitor monitor(params);
  auto in = healthyInputs();
  in.loc_status = kStatusLost;
  in.loc_ok = false;

  // Feed the same LOST reading across many ticks until the sustained
  // timer trips — dt accumulates inside the monitor across calls.
  uav_safety::SafetyOutputs out;
  for (int i = 0; i < 15; ++i) {
    out = monitor.update(in, 0.1);   // 15 * 0.1s = 1.5s > 1.0s timeout
  }

  EXPECT_TRUE(out.cmd_valid) << "LAND must be an explicit, authoritative command";
  EXPECT_EQ(out.cmd_mode, kModeLand);
  EXPECT_EQ(out.overall_level, kLevelFailsafe);
  const auto & faults = out.active_faults;
  EXPECT_NE(std::find(faults.begin(), faults.end(), "SUSTAINED_LOSS_LANDING"), faults.end());
}

TEST(SafetyMonitor, RecoveryResetsSustainedLossTimer) {
  SafetyMonitor::Params params;
  params.lost_hold_timeout_s = 1.0;
  SafetyMonitor monitor(params);
  auto bad = healthyInputs();
  bad.loc_status = kStatusLost;
  bad.loc_ok = false;

  for (int i = 0; i < 8; ++i) {   // 0.8s of loss, under the 1.0s timeout
    monitor.update(bad, 0.1);
  }
  monitor.update(healthyInputs(), 0.1);   // recovers

  // Loss resumes; if the timer didn't reset, this would already exceed
  // the cumulative 1.0s and land immediately instead of holding first.
  uav_safety::SafetyOutputs out;
  for (int i = 0; i < 5; ++i) {   // only 0.5s of the resumed loss
    out = monitor.update(bad, 0.1);
  }
  EXPECT_EQ(out.cmd_mode, kModeHold) << "sustained-loss timer must reset on recovery";
}

TEST(SafetyMonitor, NoInputsYieldsHoldNotCrash) {
  SafetyMonitor monitor;
  const SafetyInputs in;   // everything default/absent
  const auto out = monitor.update(in, 0.05);
  EXPECT_FALSE(out.cmd_valid);
  EXPECT_EQ(out.cmd_mode, kModeHold);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
