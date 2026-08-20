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

// RealMission: the real Mission Manager producer. Same topic/QoS as
// MockMission (drop-in swap per docs/DEVELOPMENT.md), backed by the
// ROS-free MissionManager state machine (see mission_manager.hpp and
// docs/MISSION.md).
//
// "Loaded from a mission file", per the Mission contract's own producer
// comment: here that means a ROS params file (parallel arrays —
// waypoint_x/y/z/yaw/acceptance_radius/type, all the same length) rather
// than inventing a bespoke mission file format. See config/ for examples.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/mission.hpp"
#include "uav_interfaces/msg/waypoint.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/planner_status.hpp"

#include "uav_mission/mission_manager.hpp"

using namespace std::chrono_literals;
using uav_interfaces::msg::Mission;
using uav_interfaces::msg::Waypoint;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::PlannerStatus;

class RealMission : public rclcpp::Node
{
public:
  RealMission()
  : Node("real_mission")
  {
    manager_ = std::make_unique<uav_mission::MissionManager>(loadMissionSpec());

    rclcpp::QoS mission_qos(rclcpp::KeepLast(1));
    mission_qos.reliable().transient_local();
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
    sensor_qos.best_effort();

    mission_pub_ = create_publisher<Mission>("/mission/current", mission_qos);

    loc_sub_ = create_subscription<LocalizationState>(
      "/localization/state", sensor_qos,
      std::bind(&RealMission::onLocalization, this, std::placeholders::_1));
    status_sub_ = create_subscription<PlannerStatus>(
      "/planning/status", sensor_qos,
      [this](const PlannerStatus::SharedPtr msg) {
        RCLCPP_DEBUG(
          get_logger(), "planner state=%u progress=%.2f", msg->state, msg->progress);
      });

    publishMission();
    // Redundant with transient_local's late-joiner delivery, but keeps a
    // heartbeat going the same way MockMission did — harmless, and useful
    // if a subscriber's QoS ever drifts from matching transient_local.
    timer_ = create_wall_timer(1s, std::bind(&RealMission::publishMission, this));
  }

private:
  uav_mission::MissionSpec loadMissionSpec()
  {
    uav_mission::MissionSpec spec;
    spec.mission_id = declare_parameter<std::string>("mission_id", "golden-scenario-01");
    spec.max_speed = static_cast<float>(declare_parameter<double>("max_speed", 3.0));
    spec.boundary_radius = static_cast<float>(declare_parameter<double>("boundary_radius", 50.0));
    spec.min_altitude = static_cast<float>(declare_parameter<double>("min_altitude", 1.0));
    spec.max_altitude = static_cast<float>(declare_parameter<double>("max_altitude", 20.0));

    const std::vector<double> default_x = {10.0};
    const std::vector<double> default_y = {0.0};
    const std::vector<double> default_z = {3.0};
    const std::vector<double> default_yaw = {std::numeric_limits<double>::quiet_NaN()};
    const std::vector<double> default_radius = {0.5};
    const std::vector<int64_t> default_type = {Waypoint::TYPE_TARGET};

    const auto x = declare_parameter<std::vector<double>>("waypoint_x", default_x);
    const auto y = declare_parameter<std::vector<double>>("waypoint_y", default_y);
    const auto z = declare_parameter<std::vector<double>>("waypoint_z", default_z);
    const auto yaw = declare_parameter<std::vector<double>>("waypoint_yaw", default_yaw);
    const auto radius =
      declare_parameter<std::vector<double>>("waypoint_acceptance_radius", default_radius);
    const auto type = declare_parameter<std::vector<int64_t>>("waypoint_type", default_type);

    const size_t n = x.size();
    if (y.size() != n || z.size() != n || yaw.size() != n ||
      radius.size() != n || type.size() != n)
    {
      RCLCPP_ERROR(
        get_logger(),
        "waypoint_* parameters have mismatched lengths — publishing an empty mission "
        "rather than a wrong one. Check your mission config file.");
      return spec;
    }

    spec.waypoints.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      uav_mission::WaypointSpec wp;
      wp.position = Eigen::Vector3d(x[i], y[i], z[i]);
      wp.yaw = static_cast<float>(yaw[i]);
      wp.acceptance_radius = static_cast<float>(radius[i]);
      wp.type = static_cast<uint8_t>(type[i]);
      spec.waypoints.push_back(wp);
    }
    return spec;
  }

  void onLocalization(const LocalizationState::SharedPtr msg)
  {
    const Eigen::Vector3d position(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (manager_->updatePosition(position, msg->localization_ok)) {
      RCLCPP_INFO(
        get_logger(), "waypoint reached, %zu leg(s) remaining",
        manager_->current().waypoints.size());
      publishMission();
    }
  }

  void publishMission()
  {
    const auto & spec = manager_->current();
    Mission m;
    m.header.stamp = this->now();
    m.header.frame_id = "map";
    m.mission_id = spec.mission_id;
    m.max_speed = spec.max_speed;
    m.boundary_radius = spec.boundary_radius;
    m.min_altitude = spec.min_altitude;
    m.max_altitude = spec.max_altitude;
    m.waypoints.reserve(spec.waypoints.size());
    for (const auto & wp : spec.waypoints) {
      Waypoint w;
      w.position.x = wp.position.x();
      w.position.y = wp.position.y();
      w.position.z = wp.position.z();
      w.yaw = wp.yaw;
      w.acceptance_radius = wp.acceptance_radius;
      w.waypoint_type = wp.type;
      m.waypoints.push_back(w);
    }
    mission_pub_->publish(m);
  }

  rclcpp::Publisher<Mission>::SharedPtr mission_pub_;
  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<uav_mission::MissionManager> manager_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealMission>());
  rclcpp::shutdown();
  return 0;
}
