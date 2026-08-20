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

#include "uav_planning/boundary_checker.hpp"

using uav_planning::BoundaryChecker;
using uav_planning::MissionBounds;

TEST(BoundaryChecker, WithinBoundsPasses) {
  MissionBounds bounds;
  bounds.geofence_center = Eigen::Vector3d(0.0, 0.0, 0.0);
  bounds.boundary_radius = 10.0;
  bounds.min_altitude = 1.0;
  bounds.max_altitude = 20.0;
  BoundaryChecker checker(bounds);

  EXPECT_TRUE(checker.isWithinBounds(Eigen::Vector3d(3.0, 4.0, 5.0)));   // dist=5, alt=5
}

TEST(BoundaryChecker, OutsideGeofenceFails) {
  MissionBounds bounds;
  bounds.geofence_center = Eigen::Vector3d(0.0, 0.0, 0.0);
  bounds.boundary_radius = 10.0;
  bounds.min_altitude = 1.0;
  bounds.max_altitude = 20.0;
  BoundaryChecker checker(bounds);

  EXPECT_FALSE(checker.isWithinBounds(Eigen::Vector3d(20.0, 0.0, 5.0)));
}

TEST(BoundaryChecker, BelowMinAltitudeFails) {
  MissionBounds bounds;
  bounds.min_altitude = 2.0;
  bounds.max_altitude = 20.0;
  BoundaryChecker checker(bounds);

  EXPECT_FALSE(checker.isWithinBounds(Eigen::Vector3d(0.0, 0.0, 1.0)));
}

TEST(BoundaryChecker, AboveMaxAltitudeFails) {
  MissionBounds bounds;
  bounds.min_altitude = 1.0;
  bounds.max_altitude = 20.0;
  BoundaryChecker checker(bounds);

  EXPECT_FALSE(checker.isWithinBounds(Eigen::Vector3d(0.0, 0.0, 25.0)));
}

TEST(BoundaryChecker, ClampProjectsBackOntoGeofenceCircle) {
  MissionBounds bounds;
  bounds.geofence_center = Eigen::Vector3d(0.0, 0.0, 0.0);
  bounds.boundary_radius = 10.0;
  bounds.min_altitude = 0.0;
  bounds.max_altitude = 50.0;
  BoundaryChecker checker(bounds);

  const Eigen::Vector3d clamped = checker.clamp(Eigen::Vector3d(20.0, 0.0, 5.0));
  EXPECT_NEAR(clamped.x(), 10.0, 1e-6);
  EXPECT_NEAR(clamped.y(), 0.0, 1e-6);
  EXPECT_TRUE(checker.isWithinBounds(clamped));
}

TEST(BoundaryChecker, ClampClipsAltitude) {
  MissionBounds bounds;
  bounds.min_altitude = 1.0;
  bounds.max_altitude = 20.0;
  BoundaryChecker checker(bounds);

  const Eigen::Vector3d clamped = checker.clamp(Eigen::Vector3d(0.0, 0.0, 100.0));
  EXPECT_NEAR(clamped.z(), 20.0, 1e-6);
}

TEST(BoundaryChecker, ZeroRadiusMeansNoHorizontalLimit) {
  MissionBounds bounds;
  bounds.boundary_radius = 0.0;
  bounds.min_altitude = 0.0;
  bounds.max_altitude = 100.0;
  BoundaryChecker checker(bounds);

  EXPECT_TRUE(checker.isWithinBounds(Eigen::Vector3d(10000.0, 10000.0, 50.0)));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
