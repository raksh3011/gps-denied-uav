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

#include "uav_planning/boundary_checker.hpp"

#include <algorithm>

namespace uav_planning
{

BoundaryChecker::BoundaryChecker(const MissionBounds & bounds)
: bounds_(bounds)
{
}

bool BoundaryChecker::isWithinBounds(const Eigen::Vector3d & position) const
{
  if (position.z() < bounds_.min_altitude || position.z() > bounds_.max_altitude) {
    return false;
  }
  if (bounds_.boundary_radius > 0.0) {
    const Eigen::Vector2d horizontal(
      position.x() - bounds_.geofence_center.x(),
      position.y() - bounds_.geofence_center.y());
    if (horizontal.norm() > bounds_.boundary_radius) {
      return false;
    }
  }
  return true;
}

Eigen::Vector3d BoundaryChecker::clamp(const Eigen::Vector3d & position) const
{
  Eigen::Vector3d out = position;
  out.z() = std::clamp(out.z(), bounds_.min_altitude, bounds_.max_altitude);

  if (bounds_.boundary_radius > 0.0) {
    Eigen::Vector2d horizontal(
      out.x() - bounds_.geofence_center.x(),
      out.y() - bounds_.geofence_center.y());
    const double dist = horizontal.norm();
    if (dist > bounds_.boundary_radius) {
      horizontal *= (bounds_.boundary_radius / dist);
      out.x() = bounds_.geofence_center.x() + horizontal.x();
      out.y() = bounds_.geofence_center.y() + horizontal.y();
    }
  }
  return out;
}

}  // namespace uav_planning
