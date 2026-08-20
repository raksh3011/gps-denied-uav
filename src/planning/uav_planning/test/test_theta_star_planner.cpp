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
#include "uav_planning/theta_star_planner.hpp"

using uav_planning::AStarPlanner;
using uav_planning::Grid3D;
using uav_planning::ObstacleSphere;
using uav_planning::ThetaStarPlanner;

namespace
{
double pathLength(const std::vector<Eigen::Vector3d> & path)
{
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}
}  // namespace

TEST(ThetaStarPlanner, FindsStraightPathOnEmptyGrid) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  ThetaStarPlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  const auto path = planner.plan(grid, start, goal);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.front() - start).norm(), 0.0, 0.5);
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(ThetaStarPlanner, ProducesShorterOrEqualPathThanGridAStarOnDiagonal) {
  // A diagonal goal is exactly where 6-connected A* is forced into a
  // staircase (only axis-aligned moves), while Theta*'s line-of-sight
  // shortcut can cut straight across — this is the whole point of using
  // Theta* for the global planner.
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 30, 30, 4);

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(5.0, 5.0, 1.0);

  AStarPlanner astar;
  ThetaStarPlanner theta;
  const auto astar_path = astar.plan(grid, start, goal);
  const auto theta_path = theta.plan(grid, start, goal);

  ASSERT_FALSE(astar_path.empty());
  ASSERT_FALSE(theta_path.empty());

  const double astar_length = pathLength(astar_path);
  const double theta_length = pathLength(theta_path);
  EXPECT_LE(theta_length, astar_length + 1e-6);

  // On a fully open diagonal, Theta* should get close to the true
  // Euclidean distance (~7.07m for a 5,5 diagonal) — the whole reason to
  // prefer it over a blocky grid path (~10m of staircase moves).
  const double euclidean = (goal - start).norm();
  EXPECT_LT(theta_length, euclidean * 1.1);
}

TEST(ThetaStarPlanner, RoutesAroundAHardObstacle) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(1.5, 0.0, 1.0), 0.8}};
  grid.inflateObstacles(obstacles, 0.2, 0.5, 5.0);

  ThetaStarPlanner planner;
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  const auto path = planner.plan(grid, start, goal);

  ASSERT_FALSE(path.empty());
  // Check every sampled sub-segment (not just the sparse waypoints) stays
  // clear of the obstacle, since any-angle segments can be long.
  for (size_t i = 1; i < path.size(); ++i) {
    const int samples = 20;
    for (int s = 0; s <= samples; ++s) {
      const double t = static_cast<double>(s) / samples;
      const Eigen::Vector3d point = path[i - 1] + t * (path[i] - path[i - 1]);
      const double dist_to_obstacle = (point - obstacles.front().center).norm();
      EXPECT_GT(dist_to_obstacle, obstacles.front().radius * 0.95)
        << "segment sample (" << point.transpose() << ") lands inside the obstacle";
    }
  }
}

TEST(ThetaStarPlanner, FindsPathOnLargeOpenGridWithSparseObstacles) {
  // Regression test for a real bug: the open-list had no stale-entry
  // check, so on a large grid with wide-open free space, cells got
  // revised (re-pushed) many times and the queue filled with stale
  // duplicates — popping one re-ran a full neighbor expansion (each with
  // an O(distance) traceLine raycast) for no benefit, burning through
  // max_expansions before ever reaching a goal that was trivially
  // reachable. This grid/obstacle layout is scaled down from the one
  // that surfaced it (a 30x15x10m arena with 8 pillars) but keeps the
  // same character: a big mostly-open grid, sparse well-separated
  // obstacles, start and goal several times the obstacle spacing apart.
  Grid3D grid(0.25, Eigen::Vector3d(-15.0, -7.5, -5.0), 120, 60, 40);
  std::vector<ObstacleSphere> obstacles = {
    {Eigen::Vector3d(-6.0, 1.5, 1.25), 0.5},
    {Eigen::Vector3d(-6.0, -3.5, 1.25), 0.5},
    {Eigen::Vector3d(-2.0, -1.0, 1.25), 0.6},
    {Eigen::Vector3d(-2.0, 4.0, 1.25), 0.4},
    {Eigen::Vector3d(2.0, 1.0, 1.25), 0.5},
    {Eigen::Vector3d(2.0, -4.0, 1.25), 0.4},
    {Eigen::Vector3d(6.0, -1.5, 1.25), 0.5},
    {Eigen::Vector3d(6.0, 3.0, 1.25), 0.5},
  };
  grid.inflateObstacles(obstacles, 0.3, 1.5, 5.0);

  ThetaStarPlanner theta;
  const auto theta_path = theta.plan(
    grid, Eigen::Vector3d(-10.0, 0.0, 1.5), Eigen::Vector3d(10.0, 0.0, 3.0));
  EXPECT_FALSE(theta_path.empty()) << "a clearly-open path exists but Theta* reported none";

  AStarPlanner astar;
  const auto astar_path = astar.plan(
    grid, Eigen::Vector3d(-10.0, 0.0, 1.5), Eigen::Vector3d(10.0, 0.0, 3.0));
  EXPECT_FALSE(astar_path.empty()) << "a clearly-open path exists but A* reported none";
}

TEST(ThetaStarPlanner, ReturnsEmptyWhenGoalIsUnreachable) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  ThetaStarPlanner planner;
  EXPECT_TRUE(
    planner.plan(grid, Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(100.0, 100.0, 100.0))
    .empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
