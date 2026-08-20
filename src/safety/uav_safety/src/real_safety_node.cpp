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

// RealSafety: the real Safety Supervisor producer. Same topics/QoS as
// MockSafety (drop-in swap, see docs/DEVELOPMENT.md), backed by the
// ROS-free SafetyMonitor state machine (see safety_monitor.hpp and
// docs/SAFETY.md for the full design rationale). Unlike MockSafety, this
// consumes every topic the frozen contract lists Safety as a subscriber
// of (docs/INTERFACES.md): LocalizationState, LocalMap, ObstacleSet,
// Trajectory, PlannerStatus.
#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/trajectory.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"
#include "uav_interfaces/msg/planner_status.hpp"
#include "uav_interfaces/msg/vehicle_command.hpp"
#include "uav_interfaces/msg/system_health.hpp"

#include "uav_safety/safety_monitor.hpp"

using uav_interfaces::msg::Trajectory;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::LocalMap;
using uav_interfaces::msg::ObstacleSet;
using uav_interfaces::msg::PlannerStatus;
using uav_interfaces::msg::VehicleCommand;
using uav_interfaces::msg::SystemHealth;
using Clock = std::chrono::steady_clock;

namespace
{
constexpr int kNearPointsChecked = 5;
}  // namespace

class RealSafety : public rclcpp::Node
{
public:
  RealSafety()
  : Node("real_safety")
  {
    uav_safety::SafetyMonitor::Params params;
    params.staleness_timeout_s = declare_parameter<double>("staleness_timeout_s", 0.5);
    params.map_staleness_timeout_s = declare_parameter<double>("map_staleness_timeout_s", 1.0);
    params.lost_hold_timeout_s = declare_parameter<double>("lost_hold_timeout_s", 3.0);
    params.min_obstacle_clearance_m =
      declare_parameter<double>("min_obstacle_clearance_m", 0.3);
    monitor_ = std::make_unique<uav_safety::SafetyMonitor>(params);

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();

    traj_sub_ = create_subscription<Trajectory>(
      "/planning/trajectory", qos,
      [this](const Trajectory::SharedPtr msg) {
        last_traj_ = *msg;
        last_traj_time_ = Clock::now();
      });
    loc_sub_ = create_subscription<LocalizationState>(
      "/localization/state", qos,
      [this](const LocalizationState::SharedPtr msg) {
        last_loc_ = *msg;
        last_loc_time_ = Clock::now();
      });
    map_sub_ = create_subscription<LocalMap>(
      "/world_model/local_map", qos,
      [this](const LocalMap::SharedPtr msg) {
        last_map_ = *msg;
        last_map_time_ = Clock::now();
      });
    obstacle_sub_ = create_subscription<ObstacleSet>(
      "/world_model/obstacles", qos,
      [this](const ObstacleSet::SharedPtr msg) {last_obstacles_ = *msg;});
    planner_status_sub_ = create_subscription<PlannerStatus>(
      "/planning/status", qos,
      [this](const PlannerStatus::SharedPtr msg) {last_planner_status_ = *msg;});

    cmd_pub_ = create_publisher<VehicleCommand>("/safety/vehicle_command", qos);
    health_pub_ = create_publisher<SystemHealth>("/safety/system_health", qos);

    const double rate_hz = declare_parameter<double>("rate_hz", 20.0);
    tick_period_s_ = 1.0 / rate_hz;
    auto period = std::chrono::duration<double>(tick_period_s_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RealSafety::tick, this));
  }

private:
  static double ageSeconds(
    const std::optional<Clock::time_point> & t, const Clock::time_point & now)
  {
    if (!t.has_value()) {return 1e9;}
    return std::chrono::duration<double>(now - *t).count();
  }

  void tick()
  {
    const auto now = Clock::now();
    const rclcpp::Time stamp = this->now();

    uav_safety::SafetyInputs in;

    in.loc_present = last_loc_.has_value();
    if (in.loc_present) {
      in.loc_ok = last_loc_->localization_ok;
      in.loc_status = last_loc_->status;
    }
    in.loc_age_s = ageSeconds(last_loc_time_, now);

    in.traj_present = last_traj_.has_value();
    if (in.traj_present) {
      in.traj_valid = last_traj_->valid;
      if (!last_traj_->points.empty()) {
        const auto & target = last_traj_->points.back();
        in.traj_target_position = Eigen::Vector3d(
          target.position.x, target.position.y, target.position.z);
        in.traj_target_yaw = target.yaw;
      }
      const int n = std::min(kNearPointsChecked, static_cast<int>(last_traj_->points.size()));
      in.traj_near_points.reserve(static_cast<size_t>(n));
      for (int i = 0; i < n; ++i) {
        const auto & p = last_traj_->points[static_cast<size_t>(i)].position;
        in.traj_near_points.emplace_back(p.x, p.y, p.z);
      }
    }
    in.traj_age_s = ageSeconds(last_traj_time_, now);

    in.map_present = last_map_.has_value();
    if (in.map_present) {in.map_valid = last_map_->map_valid;}
    in.map_age_s = ageSeconds(last_map_time_, now);

    in.planner_present = last_planner_status_.has_value();
    if (in.planner_present) {in.planner_state = last_planner_status_->state;}

    if (last_obstacles_.has_value()) {
      in.obstacles.reserve(last_obstacles_->obstacles.size());
      for (const auto & obstacle : last_obstacles_->obstacles) {
        uav_safety::ObstacleReading reading;
        reading.position = Eigen::Vector3d(
          obstacle.position.x, obstacle.position.y, obstacle.position.z);
        reading.radius = obstacle.radius;
        in.obstacles.push_back(reading);
      }
    }

    const auto out = monitor_->update(in, tick_period_s_);

    VehicleCommand cmd;
    cmd.header.stamp = stamp;
    cmd.mode = out.cmd_mode;
    cmd.position.x = out.cmd_position.x();
    cmd.position.y = out.cmd_position.y();
    cmd.position.z = out.cmd_position.z();
    cmd.yaw = out.cmd_yaw;
    cmd.valid = out.cmd_valid;
    cmd_pub_->publish(cmd);

    SystemHealth health;
    health.header.stamp = stamp;
    health.overall_level = out.overall_level;
    health.localization_level = out.localization_level;
    health.world_model_level = out.world_model_level;
    health.planning_level = out.planning_level;
    health.vehicle_link_level = out.vehicle_link_level;
    health.active_faults = out.active_faults;
    health_pub_->publish(health);
  }

  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Subscription<LocalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<ObstacleSet>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr planner_status_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<SystemHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<uav_safety::SafetyMonitor> monitor_;

  std::optional<Trajectory> last_traj_;
  std::optional<Clock::time_point> last_traj_time_;
  std::optional<LocalizationState> last_loc_;
  std::optional<Clock::time_point> last_loc_time_;
  std::optional<LocalMap> last_map_;
  std::optional<Clock::time_point> last_map_time_;
  std::optional<ObstacleSet> last_obstacles_;
  std::optional<PlannerStatus> last_planner_status_;

  double tick_period_s_{0.05};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSafety>());
  rclcpp::shutdown();
  return 0;
}
