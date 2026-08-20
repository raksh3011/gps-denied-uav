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

#include "uav_planning/grid3d.hpp"

using uav_planning::Grid3D;
using uav_planning::GridIndex;
using uav_planning::ObstacleSphere;

TEST(Grid3D, WorldToIndexRoundTrip) {
  Grid3D grid(0.2, Eigen::Vector3d(-1.0, -1.0, 0.0), 10, 10, 10);
  const GridIndex idx = grid.worldToIndex(Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_EQ(idx.x, 5);
  EXPECT_EQ(idx.y, 5);
  EXPECT_EQ(idx.z, 0);
}

TEST(Grid3D, OutOfBoundsIsTreatedAsOccupied) {
  Grid3D grid(0.2, Eigen::Vector3d(0.0, 0.0, 0.0), 5, 5, 5);
  EXPECT_TRUE(grid.isOccupied(GridIndex{-1, 0, 0}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{100, 0, 0}));
}

TEST(Grid3D, LoadOccupancyMarksCellsOccupied) {
  Grid3D grid(1.0, Eigen::Vector3d(0.0, 0.0, 0.0), 2, 2, 1);
  std::vector<uint8_t> occ = {0, 1, 0, 0};   // index (1,0,0) occupied
  grid.loadOccupancy(occ);
  EXPECT_FALSE(grid.isOccupied(GridIndex{0, 0, 0}));
  EXPECT_TRUE(grid.isOccupied(GridIndex{1, 0, 0}));
}

TEST(Grid3D, InflateObstaclesHardOccupiesNearCells) {
  Grid3D grid(0.5, Eigen::Vector3d(-2.5, -2.5, 0.0), 10, 10, 1);
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(0.0, 0.0, 0.25), 0.5}};
  grid.inflateObstacles(obstacles, 0.2, 1.0, 5.0);

  // Cell at the obstacle center must be hard-occupied.
  const GridIndex center_idx = grid.worldToIndex(Eigen::Vector3d(0.0, 0.0, 0.25));
  EXPECT_TRUE(grid.isOccupied(center_idx));

  // Far corner of the grid should be unaffected.
  EXPECT_FALSE(grid.isOccupied(GridIndex{9, 9, 0}));
}

TEST(Grid3D, InflateObstaclesAddsSoftCostBeyondHardMargin) {
  Grid3D grid(0.5, Eigen::Vector3d(-2.5, -2.5, 0.0), 10, 10, 1);
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(0.0, 0.0, 0.25), 0.5}};
  grid.inflateObstacles(obstacles, 0.2, 1.0, 5.0);

  // A cell just outside the hard margin should carry positive soft cost.
  const GridIndex near_idx = grid.worldToIndex(Eigen::Vector3d(1.0, 0.0, 0.25));
  EXPECT_GT(grid.traversalCost(near_idx), 0.0);

  // Far away, cost should be zero.
  const GridIndex far_idx = grid.worldToIndex(Eigen::Vector3d(4.0, 4.0, 0.25));
  EXPECT_DOUBLE_EQ(grid.traversalCost(far_idx), 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
