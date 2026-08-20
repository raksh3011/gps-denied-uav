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

// MissionManager: the real Mission Manager's state machine. ROS-free
// (same pattern as uav_planning_core/uav_world_model_core/uav_safety_core
// /uav_vehicle_core) so it's unit-testable without a ROS runtime —
// real_mission_node is the only place this gets wired to ROS topics.
//
// Why this exists: real_planner_node only ever reads
// `mission.waypoints.front()` — it has no concept of a multi-leg route.
// Sequencing a route is therefore Mission's job, not Planning's: this
// class watches the vehicle's own position against the current front
// waypoint's `acceptance_radius` (the field the frozen Waypoint contract
// defines for exactly this purpose) and pops it once reached, so
// real_mission_node can republish a shorter Mission with the next leg's
// waypoint now at the front. See docs/MISSION.md for the full design and
// the one known rough edge (what Planning does with an empty-waypoints
// Mission once the route completes).
#ifndef UAV_MISSION__MISSION_MANAGER_HPP_
#define UAV_MISSION__MISSION_MANAGER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace uav_mission
{

// Mirrors uav_interfaces::msg::Waypoint's fields exactly (plain types,
// no ROS message dependency in this ROS-free core).
struct WaypointSpec
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  float yaw{0.0F};              // NaN = don't-care, per the Waypoint contract
  float acceptance_radius{0.5F};
  uint8_t type{0};               // Waypoint::TYPE_*
};

// Mirrors uav_interfaces::msg::Mission's fields exactly.
struct MissionSpec
{
  std::string mission_id;
  std::vector<WaypointSpec> waypoints;   // front = current target
  float max_speed{3.0F};
  float boundary_radius{50.0F};
  float min_altitude{1.0F};
  float max_altitude{20.0F};
};

class MissionManager
{
public:
  explicit MissionManager(MissionSpec initial);

  // Call whenever a fresh LocalizationState arrives. If the vehicle is
  // within the current front waypoint's acceptance_radius, pops it and
  // returns true (caller should republish the now-shorter Mission).
  // Does nothing (returns false) if localization isn't healthy — an
  // untrustworthy position must never advance the mission — or if the
  // mission is already complete (no waypoints left).
  bool updatePosition(const Eigen::Vector3d & position, bool localization_ok);

  const MissionSpec & current() const {return spec_;}
  bool isComplete() const {return spec_.waypoints.empty();}

private:
  MissionSpec spec_;
};

}  // namespace uav_mission

#endif  // UAV_MISSION__MISSION_MANAGER_HPP_
