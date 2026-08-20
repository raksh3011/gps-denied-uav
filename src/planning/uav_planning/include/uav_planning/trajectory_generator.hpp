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

// TrajectoryGenerator: turns an ordered list of waypoints (as produced by
// AStarPlanner) into a time-parameterized uav_interfaces::msg::Trajectory,
// per the Trajectory contract in docs/INTERFACES.md. Constant-speed
// segments for v1 — no acceleration limiting/smoothing yet, see
// docs/PLANNING.md for what's still ahead.
#ifndef UAV_PLANNING__TRAJECTORY_GENERATOR_HPP_
#define UAV_PLANNING__TRAJECTORY_GENERATOR_HPP_

#include <vector>

#include <Eigen/Core>

#include "uav_interfaces/msg/trajectory_point.hpp"

namespace uav_planning
{

class TrajectoryGenerator
{
public:
  // max_speed_mps: from Mission.max_speed. waypoint_spacing_m: if consecutive
  // waypoints from the planner (typically one per grid cell) are denser
  // than this, they're thinned first so the resulting Trajectory isn't
  // needlessly long — points still land exactly on the original path,
  // just not on every single grid cell.
  TrajectoryGenerator(double max_speed_mps, double waypoint_spacing_m = 0.5);

  std::vector<uav_interfaces::msg::TrajectoryPoint> generate(
    const std::vector<Eigen::Vector3d> & waypoints) const;

private:
  double max_speed_mps_;
  double waypoint_spacing_m_;
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__TRAJECTORY_GENERATOR_HPP_
