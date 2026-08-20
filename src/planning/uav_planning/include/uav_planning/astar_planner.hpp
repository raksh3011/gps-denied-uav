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

// AStarPlanner: 6-connected 3D grid A* over a Grid3D. Global planner + the
// "cost function" piece of Person 3's ownership — edge cost is grid step
// distance plus Grid3D's soft obstacle-proximity cost, not just distance.
#ifndef UAV_PLANNING__ASTAR_PLANNER_HPP_
#define UAV_PLANNING__ASTAR_PLANNER_HPP_

#include <vector>

#include <Eigen/Core>

#include "uav_planning/grid3d.hpp"

namespace uav_planning
{

class AStarPlanner
{
public:
  // max_expansions bounds worst-case search effort so a single planning
  // tick can't blow through its time budget on a huge/maze-like map.
  explicit AStarPlanner(size_t max_expansions = 200000);

  // Returns a path of world-frame waypoints from `start` to `goal`
  // (inclusive), or an empty vector if no path was found (goal unreachable,
  // start/goal occupied, or the search budget was exhausted).
  std::vector<Eigen::Vector3d> plan(
    const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const;

private:
  size_t max_expansions_;
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__ASTAR_PLANNER_HPP_
