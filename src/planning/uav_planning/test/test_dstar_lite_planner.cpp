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

#include "uav_planning/dstar_lite_planner.hpp"

using uav_planning::DStarLitePlanner;
using uav_planning::Grid3D;
using uav_planning::ObstacleSphere;

TEST(DStarLitePlanner, NotInitializedReturnsEmpty) {
  DStarLitePlanner planner;
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  EXPECT_TRUE(planner.update(grid, Eigen::Vector3d(0.0, 0.0, 1.0)).empty());
}

TEST(DStarLitePlanner, FindsInitialPathOnEmptyGrid) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);

  const auto path = planner.update(grid, start);
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.front() - start).norm(), 0.0, 0.5);
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, ReroutesAroundANewObstacleWithoutReinitializing) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);

  const auto initial_path = planner.update(grid, start);
  ASSERT_FALSE(initial_path.empty());
  // On an empty grid the initial path should hug the direct line.
  for (const auto & point : initial_path) {
    EXPECT_LT(std::abs(point.y()), 0.5);
  }

  // Now an obstacle appears directly on that line — same Grid3D object
  // mutated in place (mirrors how real_planner rebuilds a fresh Grid3D
  // each tick from the latest LocalMap/ObstacleSet), and we call update()
  // again WITHOUT calling initialize() — this is the property under test.
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(1.5, 0.0, 1.0), 0.8}};
  grid.inflateObstacles(obstacles, 0.2, 0.5, 5.0);

  const auto rerouted_path = planner.update(grid, start);
  ASSERT_FALSE(rerouted_path.empty());
  for (const auto & point : rerouted_path) {
    const double dist_to_obstacle = (point - obstacles.front().center).norm();
    EXPECT_GT(dist_to_obstacle, obstacles.front().radius)
      << "rerouted path point (" << point.transpose() << ") lands inside the new obstacle";
  }
}

TEST(DStarLitePlanner, TracksVehicleMovingTowardGoal) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(4.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);
  planner.update(grid, start);

  // Simulate the vehicle having advanced partway toward the goal.
  const Eigen::Vector3d advanced(2.0, 0.0, 1.0);
  const auto path = planner.update(grid, advanced);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.front() - advanced).norm(), 0.0, 0.5);
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, GoalOccupiedGivesEmptyPath) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 10, 10, 4);
  DStarLitePlanner planner;
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(2.0, 0.0, 1.0);

  // Occupy the goal cell itself before initializing.
  std::vector<ObstacleSphere> obstacles = {{goal, 0.3}};
  grid.inflateObstacles(obstacles, 0.0, 0.0, 0.0);

  planner.initialize(grid, start, goal);
  EXPECT_TRUE(planner.update(grid, start).empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
