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

// ThetaStarPlanner: any-angle global planner, implemented as Lazy Theta*
// (Nash, Koenig, Tovey, "Lazy Theta*: Any-Angle Path Planning and Path
// Length Analysis in 3D", 2010 — the lazy-evaluation refinement of the
// original Theta*, Nash et al. 2007). Same grid-search skeleton as
// AStarPlanner, but a node's parent isn't restricted to a grid-adjacent
// cell: when a node is actually expanded, if its grandparent has clear
// line-of-sight to it, it re-parents through the grandparent directly.
// "Lazy" means that line-of-sight check happens once per node expanded,
// not once per candidate neighbor of every node ever pushed (the eager
// original) — most pushed candidates are superseded and never expanded,
// so eager per-neighbor checking burns the large majority of its
// raycasts on paths the search never uses. On a large open grid this
// difference is not cosmetic: see docs/PLANNING.md's note on why the
// eager version blew past its expansion budget without finding a path
// that plainly existed. On an open grid this produces paths close to the
// true Euclidean-shortest path instead of the blocky, axis-aligned
// routes 6-connected A* is limited to, without needing a separate
// post-processing smoothing pass.
#ifndef UAV_PLANNING__THETA_STAR_PLANNER_HPP_
#define UAV_PLANNING__THETA_STAR_PLANNER_HPP_

#include <Eigen/Core>

#include <vector>

#include "uav_planning/grid3d.hpp"

namespace uav_planning
{

class ThetaStarPlanner
{
public:
  explicit ThetaStarPlanner(size_t max_expansions = 200000);

  // Same contract as AStarPlanner::plan(): world-frame waypoints from
  // `start` to `goal` inclusive, or empty if none found. Unlike A*'s
  // output (one point per grid cell), consecutive waypoints here may be
  // arbitrarily far apart — each consecutive pair is a verified-clear
  // straight line, which is exactly what TrajectoryGenerator wants.
  std::vector<Eigen::Vector3d> plan(
    const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const;

private:
  size_t max_expansions_;
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__THETA_STAR_PLANNER_HPP_
