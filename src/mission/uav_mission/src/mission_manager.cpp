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

#include "uav_mission/mission_manager.hpp"

namespace uav_mission
{

MissionManager::MissionManager(MissionSpec initial)
: spec_(std::move(initial))
{
}

bool MissionManager::updatePosition(const Eigen::Vector3d & position, bool localization_ok)
{
  if (!localization_ok || spec_.waypoints.empty()) {return false;}

  const auto & wp = spec_.waypoints.front();
  const double dist = (position - wp.position).norm();
  if (dist <= static_cast<double>(wp.acceptance_radius)) {
    spec_.waypoints.erase(spec_.waypoints.begin());
    return true;
  }
  return false;
}

}  // namespace uav_mission
