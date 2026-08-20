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

#include "uav_planning/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace uav_planning
{

TrajectoryGenerator::TrajectoryGenerator(double max_speed_mps, double waypoint_spacing_m)
: max_speed_mps_(max_speed_mps), waypoint_spacing_m_(waypoint_spacing_m)
{
}

namespace
{

std::vector<Eigen::Vector3d> thin(
  const std::vector<Eigen::Vector3d> & waypoints, double spacing)
{
  if (waypoints.size() <= 2) {return waypoints;}

  std::vector<Eigen::Vector3d> thinned;
  thinned.push_back(waypoints.front());
  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    if ((waypoints[i] - thinned.back()).norm() >= spacing) {
      thinned.push_back(waypoints[i]);
    }
  }
  thinned.push_back(waypoints.back());
  return thinned;
}

}  // namespace

std::vector<uav_interfaces::msg::TrajectoryPoint> TrajectoryGenerator::generate(
  const std::vector<Eigen::Vector3d> & waypoints) const
{
  std::vector<uav_interfaces::msg::TrajectoryPoint> points;
  if (waypoints.empty()) {return points;}

  const std::vector<Eigen::Vector3d> path = thin(waypoints, waypoint_spacing_m_);
  const double speed = std::max(max_speed_mps_, 0.1);   // avoid div-by-zero on a bad Mission

  double cumulative_time = 0.0;
  float last_yaw = 0.0F;

  for (size_t i = 0; i < path.size(); ++i) {
    uav_interfaces::msg::TrajectoryPoint pt;
    pt.position.x = path[i].x();
    pt.position.y = path[i].y();
    pt.position.z = path[i].z();

    if (i + 1 < path.size()) {
      const Eigen::Vector3d delta = path[i + 1] - path[i];
      const double segment_len = delta.norm();
      Eigen::Vector3d direction = Eigen::Vector3d::Zero();
      if (segment_len > 1e-6) {
        direction = delta / segment_len;
      }

      pt.velocity.x = direction.x() * speed;
      pt.velocity.y = direction.y() * speed;
      pt.velocity.z = direction.z() * speed;

      if (segment_len > 1e-6) {
        last_yaw = static_cast<float>(std::atan2(direction.y(), direction.x()));
      }

      const double dt = segment_len / speed;
      const auto sec = static_cast<int32_t>(cumulative_time);
      const auto nanosec = static_cast<uint32_t>((cumulative_time - sec) * 1e9);
      pt.time_from_start.sec = sec;
      pt.time_from_start.nanosec = nanosec;
      cumulative_time += dt;
    } else {
      // final point: arrived, zero velocity
      pt.velocity.x = 0.0;
      pt.velocity.y = 0.0;
      pt.velocity.z = 0.0;
      const auto sec = static_cast<int32_t>(cumulative_time);
      const auto nanosec = static_cast<uint32_t>((cumulative_time - sec) * 1e9);
      pt.time_from_start.sec = sec;
      pt.time_from_start.nanosec = nanosec;
    }

    pt.yaw = last_yaw;
    points.push_back(pt);
  }

  return points;
}

}  // namespace uav_planning
