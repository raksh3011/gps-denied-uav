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

// BoundaryChecker: Mission's geofence (boundary_radius around the first
// waypoint, per Mission.msg) and altitude limits. Person 3's "boundary
// checking" ownership — kept as a standalone, ROS-free, unit-testable piece
// rather than folded into the planner node.
#ifndef UAV_PLANNING__BOUNDARY_CHECKER_HPP_
#define UAV_PLANNING__BOUNDARY_CHECKER_HPP_

#include <Eigen/Core>

namespace uav_planning
{

struct MissionBounds
{
  Eigen::Vector3d geofence_center;
  double boundary_radius{0.0};   // meters; <= 0 means "no horizontal limit"
  double min_altitude{0.0};      // meters AGL
  double max_altitude{0.0};      // meters AGL
};

class BoundaryChecker
{
public:
  explicit BoundaryChecker(const MissionBounds & bounds);

  bool isWithinBounds(const Eigen::Vector3d & position) const;

  // Clamps a candidate position to satisfy altitude limits and, if outside
  // the geofence, projects it back onto the geofence circle. Used to make
  // a goal reachable/legal before planning to it, rather than silently
  // planning to an out-of-bounds point.
  Eigen::Vector3d clamp(const Eigen::Vector3d & position) const;

private:
  MissionBounds bounds_;
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__BOUNDARY_CHECKER_HPP_
