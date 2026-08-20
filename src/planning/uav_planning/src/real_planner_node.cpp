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
// workflow) but backed by two layered real planners, matching Person 3's
// global+local ownership split:
//   - Global: ThetaStarPlanner, run once whenever the goal (or the
//     planner's own state) changes — any-angle search, so the reference
//     path is close to the true shortest path, not a blocky grid staircase.
//   - Local: DStarLitePlanner, run every tick. It holds search state
//     across ticks and only re-examines what actually changed (vehicle
//     motion, new/moved obstacles), instead of re-searching from scratch —
//     this is what actually gets published as the Trajectory, since it's
//     the one with a bounded, low-latency reaction to a changing
//     environment.
// See docs/PLANNING.md for the full design rationale and known limits
// (notably: DStarLitePlanner assumes a fixed-origin map — a genuinely
// rolling/re-centering LocalMap forces a full re-initialize, losing the
// incremental benefit for that tick; see the doc for why).
#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/mission.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"
#include "uav_interfaces/msg/trajectory.hpp"
#include "uav_interfaces/msg/planner_status.hpp"

#include "uav_planning/boundary_checker.hpp"
#include "uav_planning/dstar_lite_planner.hpp"
#include "uav_planning/grid3d.hpp"
#include "uav_planning/theta_star_planner.hpp"
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
constexpr double kGoalChangeToleranceM = 0.1;   // re-run global + re-init local beyond this
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

    const bool goal_changed = !dstar_.isInitialized() || !last_goal_.has_value() ||
      (goal - *last_goal_).norm() > kGoalChangeToleranceM;

    if (goal_changed) {
      // Global reference plan: any-angle Theta*, run once per goal. Not
      // itself published — DStarLitePlanner is re-seeded fresh against
      // the same start/goal and becomes the ongoing source of truth every
      // tick from here on. Kept for total_path_length_ (progress metric)
      // and as a sanity signal: if Theta* can't find a path either, the
      // goal is genuinely unreachable right now, not a D* Lite quirk.
      uav_planning::ThetaStarPlanner theta;
      const auto global_path = theta.plan(grid, start, goal);
      if (global_path.empty()) {
        status.state = PlannerStatus::STATE_FAILED;
        status.message = "no global path found to goal";
        status_pub_->publish(status);
        return;
      }
      total_path_length_ = 0.0;
      for (size_t i = 1; i < global_path.size(); ++i) {
        total_path_length_ += (global_path[i] - global_path[i - 1]).norm();
      }
      dstar_.initialize(grid, start, goal);
      last_goal_ = goal;
    }

    // Confidence-adaptive risk margin: feed the live localization quality
    // into the local planner every tick. Cheap no-op unless the risk band
    // actually crosses a threshold (see DStarLitePlanner header).
    dstar_.setLocalizationRisk(loc.confidence, loc.status);

    std::vector<Eigen::Vector3d> path = dstar_.update(grid, start);
    if (path.empty() && dstar_.isInitialized()) {
      // update() returns empty both for "genuinely unreachable" and for
      // "map geometry changed under us, please re-initialize" (see
      // DStarLitePlanner::update) -- isInitialized() distinguishes them:
      // it goes false in the latter case, so try exactly once more here
      // rather than reporting failure for a recoverable map re-centering.
      dstar_.initialize(grid, start, goal);
      dstar_.setLocalizationRisk(loc.confidence, loc.status);   // initialize() resets the band
      path = dstar_.update(grid, start);
    }

    if (path.empty()) {
      status.state = PlannerStatus::STATE_FAILED;
      status.message = "no local path found to goal";
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
    status.state = PlannerStatus::STATE_EXECUTING;
    status.message = "";
    status.progress = total_path_length_ > 1e-6 ?
      static_cast<float>(std::max(0.0, 1.0 - remaining / total_path_length_)) : 1.0F;
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

  uav_planning::DStarLitePlanner dstar_;
  std::optional<Eigen::Vector3d> last_goal_;
  double total_path_length_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealPlanner>());
  rclcpp::shutdown();
  return 0;
}
