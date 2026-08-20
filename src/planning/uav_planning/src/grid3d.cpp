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

#include "uav_planning/grid3d.hpp"

#include <algorithm>
#include <cmath>

namespace uav_planning
{

Grid3D::Grid3D(
  double resolution, const Eigen::Vector3d & origin,
  int size_x, int size_y, int size_z)
: resolution_(resolution), origin_(origin),
  size_x_(size_x), size_y_(size_y), size_z_(size_z),
  occupied_(static_cast<size_t>(size_x) * size_y * size_z, false),
  cost_(static_cast<size_t>(size_x) * size_y * size_z, 0.0)
{
}

size_t Grid3D::flatten(const GridIndex & idx) const
{
  return static_cast<size_t>(idx.x) +
         static_cast<size_t>(idx.y) * size_x_ +
         static_cast<size_t>(idx.z) * size_x_ * size_y_;
}

bool Grid3D::inBounds(const GridIndex & idx) const
{
  return idx.x >= 0 && idx.x < size_x_ &&
         idx.y >= 0 && idx.y < size_y_ &&
         idx.z >= 0 && idx.z < size_z_;
}

bool Grid3D::isOccupied(const GridIndex & idx) const
{
  if (!inBounds(idx)) {return true;}   // out of known map = do not route there
  return occupied_[flatten(idx)];
}

double Grid3D::traversalCost(const GridIndex & idx) const
{
  if (!inBounds(idx)) {return 0.0;}
  return cost_[flatten(idx)];
}

GridIndex Grid3D::worldToIndex(const Eigen::Vector3d & world) const
{
  const Eigen::Vector3d rel = (world - origin_) / resolution_;
  return GridIndex{
    static_cast<int>(std::floor(rel.x())),
    static_cast<int>(std::floor(rel.y())),
    static_cast<int>(std::floor(rel.z()))};
}

Eigen::Vector3d Grid3D::indexToWorld(const GridIndex & idx) const
{
  return origin_ + Eigen::Vector3d(
    (idx.x + 0.5) * resolution_,
    (idx.y + 0.5) * resolution_,
    (idx.z + 0.5) * resolution_);
}

Grid3D::LineTrace Grid3D::traceLine(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
{
  const double length = (b - a).norm();
  const double step = resolution_ * 0.5;
  const int num_steps = std::max(1, static_cast<int>(std::ceil(length / step)));

  LineTrace result;
  result.clear = true;
  result.cost = 0.0;

  for (int i = 0; i <= num_steps; ++i) {
    const double t = static_cast<double>(i) / num_steps;
    const Eigen::Vector3d sample = a + t * (b - a);
    const GridIndex idx = worldToIndex(sample);
    if (isOccupied(idx)) {
      result.clear = false;
      result.cost = 0.0;
      return result;
    }
    result.cost += traversalCost(idx);
  }
  return result;
}

void Grid3D::loadOccupancy(const std::vector<uint8_t> & occupancy)
{
  const size_t n = std::min(occupancy.size(), occupied_.size());
  for (size_t i = 0; i < n; ++i) {
    occupied_[i] = (occupancy[i] != 0);
  }
}

void Grid3D::inflateObstacles(
  const std::vector<ObstacleSphere> & obstacles,
  double hard_margin, double soft_margin, double soft_cost_weight)
{
  for (int z = 0; z < size_z_; ++z) {
    for (int y = 0; y < size_y_; ++y) {
      for (int x = 0; x < size_x_; ++x) {
        const GridIndex idx{x, y, z};
        const Eigen::Vector3d world = indexToWorld(idx);
        const size_t flat = flatten(idx);

        for (const auto & obstacle : obstacles) {
          const double dist_to_surface = (world - obstacle.center).norm() - obstacle.radius;

          if (dist_to_surface <= hard_margin) {
            occupied_[flat] = true;
          } else if (dist_to_surface <= hard_margin + soft_margin) {
            const double proximity = (hard_margin + soft_margin) - dist_to_surface;
            cost_[flat] = std::max(cost_[flat], proximity * soft_cost_weight);
          }
        }
      }
    }
  }
}

}  // namespace uav_planning
