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

#include "uav_mission/mission_manager.hpp"

using uav_mission::MissionManager;
using uav_mission::MissionSpec;
using uav_mission::WaypointSpec;

namespace
{
MissionSpec twoLegMission()
{
  MissionSpec spec;
  spec.mission_id = "test-mission";
  spec.max_speed = 3.0F;
  spec.boundary_radius = 50.0F;
  spec.min_altitude = 1.0F;
  spec.max_altitude = 20.0F;

  WaypointSpec a;
  a.position = Eigen::Vector3d(5.0, 0.0, 2.0);
  a.acceptance_radius = 0.5F;
  spec.waypoints.push_back(a);

  WaypointSpec b;
  b.position = Eigen::Vector3d(10.0, 0.0, 3.0);
  b.acceptance_radius = 0.5F;
  spec.waypoints.push_back(b);

  return spec;
}
}  // namespace

TEST(MissionManager, StartsAtFirstWaypointNotComplete) {
  MissionManager manager(twoLegMission());
  ASSERT_EQ(manager.current().waypoints.size(), 2u);
  EXPECT_EQ(manager.current().waypoints.front().position, Eigen::Vector3d(5.0, 0.0, 2.0));
  EXPECT_FALSE(manager.isComplete());
}

TEST(MissionManager, StaysOnLegWhenOutsideAcceptanceRadius) {
  MissionManager manager(twoLegMission());
  const bool changed = manager.updatePosition(Eigen::Vector3d(2.0, 0.0, 2.0), true);
  EXPECT_FALSE(changed);
  EXPECT_EQ(manager.current().waypoints.size(), 2u);
}

TEST(MissionManager, AdvancesToNextLegWithinAcceptanceRadius) {
  MissionManager manager(twoLegMission());
  const bool changed = manager.updatePosition(Eigen::Vector3d(5.1, 0.0, 2.0), true);
  EXPECT_TRUE(changed);
  ASSERT_EQ(manager.current().waypoints.size(), 1u);
  EXPECT_EQ(manager.current().waypoints.front().position, Eigen::Vector3d(10.0, 0.0, 3.0));
  EXPECT_FALSE(manager.isComplete());
}

TEST(MissionManager, CompletesAfterFinalWaypointReached) {
  MissionManager manager(twoLegMission());
  manager.updatePosition(Eigen::Vector3d(5.1, 0.0, 2.0), true);   // reach leg 1
  const bool changed = manager.updatePosition(Eigen::Vector3d(10.05, 0.0, 3.0), true);
  EXPECT_TRUE(changed);
  EXPECT_TRUE(manager.isComplete());
  EXPECT_TRUE(manager.current().waypoints.empty());
}

TEST(MissionManager, DoesNotAdvanceOnUnhealthyLocalization) {
  MissionManager manager(twoLegMission());
  // Physically within radius, but localization_ok=false — must not trust it.
  const bool changed = manager.updatePosition(Eigen::Vector3d(5.0, 0.0, 2.0), false);
  EXPECT_FALSE(changed);
  EXPECT_EQ(manager.current().waypoints.size(), 2u);
}

TEST(MissionManager, UpdatePositionOnCompletedMissionIsNoop) {
  MissionSpec spec;
  spec.mission_id = "single-leg";
  WaypointSpec wp;
  wp.position = Eigen::Vector3d(1.0, 0.0, 1.0);
  wp.acceptance_radius = 0.5F;
  spec.waypoints.push_back(wp);

  MissionManager manager(spec);
  manager.updatePosition(Eigen::Vector3d(1.0, 0.0, 1.0), true);
  ASSERT_TRUE(manager.isComplete());

  const bool changed = manager.updatePosition(Eigen::Vector3d(1.0, 0.0, 1.0), true);
  EXPECT_FALSE(changed);
  EXPECT_TRUE(manager.isComplete());
}

TEST(MissionManager, StaticFieldsPreservedAcrossAdvances) {
  MissionManager manager(twoLegMission());
  manager.updatePosition(Eigen::Vector3d(5.1, 0.0, 2.0), true);
  EXPECT_EQ(manager.current().mission_id, "test-mission");
  EXPECT_FLOAT_EQ(manager.current().max_speed, 3.0F);
  EXPECT_FLOAT_EQ(manager.current().boundary_radius, 50.0F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
