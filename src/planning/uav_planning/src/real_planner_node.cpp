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

// RealPlanner: the real Planning producer. Same topics/contract as
// MockPlanner (drop-in swap, see docs/DEVELOPMENT.md's mock-swap
// workflow) but backed by a real global planner (AStarPlanner over a
// Grid3D built from LocalMap + inflated ObstacleSet), boundary checking
// (BoundaryChecker), and trajectory generation (TrajectoryGenerator).
// Replans from scratch every tick against the latest map/obstacles —
// see docs/PLANNING.md for why that's the "dynamic replanning" strategy
// for v1, and what a real local planner would add on top.
#include <chrono>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/mission.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"
#include "uav_interfaces/msg/trajectory.hpp"
#include "uav_interfaces/msg/planner_status.hpp"

#include "uav_planning/astar_planner.hpp"
#include "uav_planning/boundary_checker.hpp"
#include "uav_planning/grid3d.hpp"
#include "uav_planning/trajectory_generator.hpp"

using uav_interfaces::msg::Mission;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::LocalMap;
using uav_interfaces::msg::ObstacleSet;
using uav_interfaces::msg::Trajectory;
using uav_interfaces::msg::PlannerStatus;

namespace
{
constexpr double kHardMarginM = 0.3;      // hard-occupied buffer beyond an obstacle's own radius
constexpr double kSoftMarginM = 1.5;      // additional soft-cost buffer beyond the hard margin
constexpr double kSoftCostWeight = 5.0;   // cost per meter of proximity within the soft margin
}  // namespace

class RealPlanner : public rclcpp::Node
{
public:
  RealPlanner()
  : Node("real_planner")
  {
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
    sensor_qos.best_effort();
    rclcpp::QoS mission_qos(rclcpp::KeepLast(1));
    mission_qos.reliable().transient_local();

    mission_sub_ = create_subscription<Mission>(
      "/mission/current", mission_qos,
      [this](const Mission::SharedPtr msg) {mission_ = *msg;});
    loc_sub_ = create_subscription<LocalizationState>(
      "/localization/state", sensor_qos,
      [this](const LocalizationState::SharedPtr msg) {loc_ = *msg;});
    map_sub_ = create_subscription<LocalMap>(
      "/world_model/local_map", sensor_qos,
      [this](const LocalMap::SharedPtr msg) {map_ = *msg;});
    obstacle_sub_ = create_subscription<ObstacleSet>(
      "/world_model/obstacles", sensor_qos,
      [this](const ObstacleSet::SharedPtr msg) {obstacles_ = *msg;});

    traj_pub_ = create_publisher<Trajectory>("/planning/trajectory", sensor_qos);
    status_pub_ = create_publisher<PlannerStatus>("/planning/status", sensor_qos);

    double rate_hz = declare_parameter<double>("rate_hz", 10.0);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RealPlanner::tick, this));
  }

private:
  void tick()
  {
    PlannerStatus status;
    status.header.stamp = this->now();

    if (!mission_.has_value() || !loc_.has_value() || !map_.has_value() || !map_->map_valid) {
      status.state = PlannerStatus::STATE_IDLE;
      status.message = "waiting for mission/localization/map";
      status_pub_->publish(status);
      return;
    }

    const auto & loc = *loc_;
    const auto & mission = *mission_;
    const auto & map = *map_;

    if (!loc.localization_ok || mission.waypoints.empty()) {
      status.state = PlannerStatus::STATE_FAILED;
      status.message = "invalid localization or empty mission";
      status_pub_->publish(status);
      return;
    }

    const Eigen::Vector3d start(
      loc.pose.position.x, loc.pose.position.y, loc.pose.position.z);
    const auto & goal_wp = mission.waypoints.front();
    const Eigen::Vector3d requested_goal(
      goal_wp.position.x, goal_wp.position.y, goal_wp.position.z);

    uav_planning::MissionBounds bounds;
    bounds.geofence_center = requested_goal;   // Mission contract: geofence centers on waypoint[0]
    bounds.boundary_radius = mission.boundary_radius;
    bounds.min_altitude = mission.min_altitude;
    bounds.max_altitude = mission.max_altitude;
    uav_planning::BoundaryChecker boundary(bounds);
    const Eigen::Vector3d goal = boundary.clamp(requested_goal);

    uav_planning::Grid3D grid(
      map.resolution,
      Eigen::Vector3d(map.origin.x, map.origin.y, map.origin.z),
      static_cast<int>(map.size_x), static_cast<int>(map.size_y), static_cast<int>(map.size_z));
    grid.loadOccupancy(map.occupancy);

    if (obstacles_.has_value()) {
      std::vector<uav_planning::ObstacleSphere> spheres;
      spheres.reserve(obstacles_->obstacles.size());
      for (const auto & obstacle : obstacles_->obstacles) {
        uav_planning::ObstacleSphere sphere;
        sphere.center = Eigen::Vector3d(
          obstacle.position.x, obstacle.position.y, obstacle.position.z);
        sphere.radius = obstacle.radius;
        spheres.push_back(sphere);
      }
      grid.inflateObstacles(spheres, kHardMarginM, kSoftMarginM, kSoftCostWeight);
    }

    uav_planning::AStarPlanner planner;
    const std::vector<Eigen::Vector3d> path = planner.plan(grid, start, goal);

    if (path.empty()) {
      status.state = PlannerStatus::STATE_FAILED;
      status.message = "no path found to goal";
      status_pub_->publish(status);
      return;
    }

    uav_planning::TrajectoryGenerator generator(mission.max_speed, map.resolution * 2.5);
    Trajectory traj;
    traj.header.stamp = status.header.stamp;
    traj.header.frame_id = "map";
    traj.points = generator.generate(path);
    traj.valid = !traj.points.empty();
    traj_pub_->publish(traj);

    const double remaining = (goal - start).norm();
    const double total = (goal - path.front()).norm();
    status.state = PlannerStatus::STATE_EXECUTING;
    status.message = "";
    status.progress = total > 1e-6 ? static_cast<float>(1.0 - remaining / total) : 1.0F;
    status_pub_->publish(status);
  }

  rclcpp::Subscription<Mission>::SharedPtr mission_sub_;
  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Subscription<LocalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<ObstacleSet>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<PlannerStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<Mission> mission_;
  std::optional<LocalizationState> loc_;
  std::optional<LocalMap> map_;
  std::optional<ObstacleSet> obstacles_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealPlanner>());
  rclcpp::shutdown();
  return 0;
}
