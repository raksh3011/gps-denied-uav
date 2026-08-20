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

#include <cmath>

#include "uav_planning/trajectory_generator.hpp"

using uav_planning::TrajectoryGenerator;

TEST(TrajectoryGenerator, EmptyWaypointsProducesEmptyTrajectory) {
  TrajectoryGenerator gen(2.0);
  EXPECT_TRUE(gen.generate({}).empty());
}

TEST(TrajectoryGenerator, FirstPointMatchesStart) {
  TrajectoryGenerator gen(2.0, 0.1);
  std::vector<Eigen::Vector3d> waypoints = {
    Eigen::Vector3d(0.0, 0.0, 1.0),
    Eigen::Vector3d(1.0, 0.0, 1.0),
    Eigen::Vector3d(2.0, 0.0, 1.0),
  };
  const auto points = gen.generate(waypoints);
  ASSERT_FALSE(points.empty());
  EXPECT_DOUBLE_EQ(points.front().position.x, 0.0);
  EXPECT_DOUBLE_EQ(points.back().position.x, 2.0);
}

TEST(TrajectoryGenerator, TimeIncreasesMonotonically) {
  TrajectoryGenerator gen(1.0, 0.1);
  std::vector<Eigen::Vector3d> waypoints = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Vector3d(3.0, 0.0, 0.0),
  };
  const auto points = gen.generate(waypoints);
  ASSERT_GE(points.size(), 2u);
  for (size_t i = 1; i < points.size(); ++i) {
    const double prev_t = points[i - 1].time_from_start.sec +
      points[i - 1].time_from_start.nanosec * 1e-9;
    const double cur_t = points[i].time_from_start.sec +
      points[i].time_from_start.nanosec * 1e-9;
    EXPECT_GE(cur_t, prev_t);
  }
}

TEST(TrajectoryGenerator, FinalPointHasZeroVelocity) {
  TrajectoryGenerator gen(2.0, 0.1);
  std::vector<Eigen::Vector3d> waypoints = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(5.0, 0.0, 0.0),
  };
  const auto points = gen.generate(waypoints);
  ASSERT_FALSE(points.empty());
  const auto & last = points.back();
  EXPECT_DOUBLE_EQ(last.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(last.velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(last.velocity.z, 0.0);
}

TEST(TrajectoryGenerator, SpeedMatchesConfiguredMaxSpeed) {
  const double max_speed = 3.0;
  TrajectoryGenerator gen(max_speed, 0.1);
  std::vector<Eigen::Vector3d> waypoints = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(10.0, 0.0, 0.0),
  };
  const auto points = gen.generate(waypoints);
  ASSERT_GE(points.size(), 1u);
  const double speed = std::sqrt(
    points.front().velocity.x * points.front().velocity.x +
    points.front().velocity.y * points.front().velocity.y +
    points.front().velocity.z * points.front().velocity.z);
  EXPECT_NEAR(speed, max_speed, 1e-6);
}

TEST(TrajectoryGenerator, ThinsDenseWaypointsButKeepsEndpoints) {
  TrajectoryGenerator gen(1.0, 1.0);   // 1m spacing threshold
  std::vector<Eigen::Vector3d> dense_waypoints;
  for (int i = 0; i <= 20; ++i) {
    dense_waypoints.emplace_back(i * 0.1, 0.0, 0.0);   // 0.1m apart, 21 points over 2m
  }
  const auto points = gen.generate(dense_waypoints);
  EXPECT_LT(points.size(), dense_waypoints.size());
  EXPECT_DOUBLE_EQ(points.front().position.x, 0.0);
  EXPECT_DOUBLE_EQ(points.back().position.x, 2.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
