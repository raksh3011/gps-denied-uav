// MockPlanner: consumes Mission + LocalizationState + LocalMap/ObstacleSet
// and publishes a trivial straight-line Trajectory plus PlannerStatus, so
// Safety/Mission/PX4-Interface can be developed independently of real
// global/local planning algorithms.
#include <chrono>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/mission.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"
#include "uav_interfaces/msg/trajectory.hpp"
#include "uav_interfaces/msg/trajectory_point.hpp"
#include "uav_interfaces/msg/planner_status.hpp"

using namespace std::chrono_literals;
using uav_interfaces::msg::Mission;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::LocalMap;
using uav_interfaces::msg::ObstacleSet;
using uav_interfaces::msg::Trajectory;
using uav_interfaces::msg::TrajectoryPoint;
using uav_interfaces::msg::PlannerStatus;

class MockPlanner : public rclcpp::Node
{
public:
  MockPlanner() : Node("mock_planner")
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
      [this](const LocalMap::SharedPtr msg) {have_map_ = msg->map_valid;});
    obstacle_sub_ = create_subscription<ObstacleSet>(
      "/world_model/obstacles", sensor_qos,
      [this](const ObstacleSet::SharedPtr) {have_obstacles_ = true;});

    traj_pub_ = create_publisher<Trajectory>("/planning/trajectory", sensor_qos);
    status_pub_ = create_publisher<PlannerStatus>("/planning/status", sensor_qos);

    double rate_hz = declare_parameter<double>("rate_hz", 10.0);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockPlanner::tick, this));
  }

private:
  void tick()
  {
    PlannerStatus status;
    status.header.stamp = this->now();

    if (!mission_.has_value() || !loc_.has_value() || !have_map_) {
      status.state = PlannerStatus::STATE_IDLE;
      status.message = "waiting for mission/localization/map";
      status_pub_->publish(status);
      return;
    }

    const auto & loc = *loc_;
    const auto & mission = *mission_;

    if (!loc.localization_ok || mission.waypoints.empty()) {
      status.state = PlannerStatus::STATE_FAILED;
      status.message = "invalid localization or empty mission";
      status_pub_->publish(status);
      return;
    }

    const auto & target = mission.waypoints.front().position;
    const auto & p0 = loc.pose.position;

    Trajectory traj;
    traj.header.stamp = status.header.stamp;
    traj.header.frame_id = "map";
    constexpr int n_points = 10;
    for (int i = 0; i <= n_points; ++i) {
      const double frac = static_cast<double>(i) / n_points;
      TrajectoryPoint pt;
      pt.time_from_start.sec = static_cast<int32_t>(frac * 5);
      pt.position.x = p0.x + frac * (target.x - p0.x);
      pt.position.y = p0.y + frac * (target.y - p0.y);
      pt.position.z = p0.z + frac * (target.z - p0.z);
      traj.points.push_back(pt);
    }
    traj.valid = true;
    traj_pub_->publish(traj);

    status.state = PlannerStatus::STATE_EXECUTING;
    status.message = "";
    status.progress = 0.0F;
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
  bool have_map_{false};
  bool have_obstacles_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockPlanner>());
  rclcpp::shutdown();
  return 0;
}
