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

// Grid3D: occupancy grid used by AStarPlanner. Deliberately independent of
// ROS — built from plain numbers, not uav_interfaces::msg::LocalMap
// directly, so it (and everything that uses it) is unit-testable without a
// ROS runtime. The real_planner node is the only place that converts
// LocalMap/ObstacleSet into a Grid3D.
#ifndef UAV_PLANNING__GRID3D_HPP_
#define UAV_PLANNING__GRID3D_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace uav_planning
{

struct GridIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

// A single obstacle to inflate into the grid: a world-frame sphere.
struct ObstacleSphere
{
  Eigen::Vector3d center;
  double radius{0.0};
};

// Occupancy + soft proximity cost over a voxel grid in the "map" frame.
// - Cells within an obstacle's radius (+ hard_margin) are hard-occupied:
//   the planner will never route through them ("terrain/map constraints").
// - Cells further out, within soft_margin of that hard boundary, carry an
//   added traversal cost that decays with distance — a real cost function,
//   not just distance, so the planner prefers standoff from obstacles
//   rather than grazing the hard boundary ("planning cost function").
class Grid3D
{
public:
  Grid3D(
    double resolution, const Eigen::Vector3d & origin,
    int size_x, int size_y, int size_z);

  // Marks LocalMap's own occupancy array as hard-occupied (0=free per the
  // LocalMap contract; anything else is treated as occupied).
  void loadOccupancy(const std::vector<uint8_t> & occupancy);

  // Inflates each obstacle: hard-occupies cells within (radius + hard_margin)
  // of the center, and adds decaying soft cost out to soft_margin beyond that.
  void inflateObstacles(
    const std::vector<ObstacleSphere> & obstacles,
    double hard_margin, double soft_margin, double soft_cost_weight);

  bool inBounds(const GridIndex & idx) const;
  bool isOccupied(const GridIndex & idx) const;
  double traversalCost(const GridIndex & idx) const;   // additive soft cost, 0 if none

  GridIndex worldToIndex(const Eigen::Vector3d & world) const;
  Eigen::Vector3d indexToWorld(const GridIndex & idx) const;

  double resolution() const {return resolution_;}
  int sizeX() const {return size_x_;}
  int sizeY() const {return size_y_;}
  int sizeZ() const {return size_z_;}

private:
  size_t flatten(const GridIndex & idx) const;

  double resolution_;
  Eigen::Vector3d origin_;
  int size_x_;
  int size_y_;
  int size_z_;
  std::vector<bool> occupied_;
  std::vector<double> cost_;
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__GRID3D_HPP_
