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

#include "uav_planning/astar_planner.hpp"
#include "uav_planning/grid3d.hpp"

using uav_planning::AStarPlanner;
using uav_planning::Grid3D;
using uav_planning::ObstacleSphere;

TEST(AStarPlanner, FindsStraightPathOnEmptyGrid) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  AStarPlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  const auto path = planner.plan(grid, start, goal);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.front() - start).norm(), 0.0, 0.5);
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(AStarPlanner, RoutesAroundAHardObstacle) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(1.5, 0.0, 1.0), 0.8}};
  grid.inflateObstacles(obstacles, 0.2, 0.5, 5.0);

  AStarPlanner planner;
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  const auto path = planner.plan(grid, start, goal);

  ASSERT_FALSE(path.empty());
  for (const auto & point : path) {
    const double dist_to_obstacle = (point - obstacles.front().center).norm();
    EXPECT_GT(dist_to_obstacle, obstacles.front().radius)
      << "path point (" << point.transpose() << ") lands inside the obstacle";
  }
}

TEST(AStarPlanner, ReturnsEmptyWhenGoalIsUnreachable) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(100.0, 100.0, 100.0);   // far outside the grid
  AStarPlanner planner;

  EXPECT_TRUE(planner.plan(grid, start, goal).empty());
}

TEST(AStarPlanner, ReturnsEmptyWhenStartIsOccupied) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(0.0, 0.0, 1.0), 0.5}};
  grid.inflateObstacles(obstacles, 0.2, 0.0, 0.0);

  AStarPlanner planner;
  const auto path = planner.plan(grid, Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(3.0, 0.0, 1.0));
  EXPECT_TRUE(path.empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
