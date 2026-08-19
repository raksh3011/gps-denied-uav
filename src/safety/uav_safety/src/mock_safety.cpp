// MockSafety: the single gate between Planning and the vehicle interface.
// Validates incoming Trajectory + LocalizationState, forwards a
// VehicleCommand only when both are healthy, and publishes SystemHealth.
// Real watchdogs and failure/recovery state machine come later; this
// proves the contract.
#include <chrono>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/trajectory.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/vehicle_command.hpp"
#include "uav_interfaces/msg/system_health.hpp"

using namespace std::chrono_literals;
using uav_interfaces::msg::Trajectory;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::VehicleCommand;
using uav_interfaces::msg::SystemHealth;
using Clock = std::chrono::steady_clock;

class MockSafety : public rclcpp::Node
{
public:
  MockSafety() : Node("mock_safety")
  {
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

    cmd_pub_ = create_publisher<VehicleCommand>("/safety/vehicle_command", qos);
    health_pub_ = create_publisher<SystemHealth>("/safety/system_health", qos);

    double rate_hz = declare_parameter<double>("rate_hz", 20.0);
    staleness_timeout_s_ = declare_parameter<double>("staleness_timeout_s", 0.5);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockSafety::tick, this));
  }

private:
  void tick()
  {
    const auto now = Clock::now();
    const auto stamp = this->now();

    auto age = [&](const std::optional<Clock::time_point> & t) {
        if (!t.has_value()) {return std::numeric_limits<double>::infinity();}
        return std::chrono::duration<double>(now - *t).count();
      };

    const bool loc_stale = !last_loc_.has_value() || age(last_loc_time_) > staleness_timeout_s_;
    const bool traj_stale = !last_traj_.has_value() || age(last_traj_time_) > staleness_timeout_s_;
    const bool loc_ok = last_loc_.has_value() && last_loc_->localization_ok;
    const bool traj_ok = last_traj_.has_value() && last_traj_->valid;

    SystemHealth health;
    health.header.stamp = stamp;

    VehicleCommand cmd;
    cmd.header.stamp = stamp;

    if (loc_stale || traj_stale || !loc_ok || !traj_ok) {
      health.overall_level = SystemHealth::LEVEL_CRITICAL;
      std::vector<std::string> faults;
      if (loc_stale) {faults.push_back("LOC_STALE");}
      if (traj_stale) {faults.push_back("TRAJ_STALE");}
      if (last_loc_.has_value() && !loc_ok) {faults.push_back("LOC_NOT_OK");}
      if (last_traj_.has_value() && !traj_ok) {faults.push_back("TRAJ_INVALID");}
      health.active_faults = faults;
      cmd.mode = VehicleCommand::MODE_HOLD;
      cmd.valid = false;
    } else {
      health.overall_level = SystemHealth::LEVEL_OK;
      const auto & target = last_traj_->points.back();
      cmd.mode = VehicleCommand::MODE_POSITION;
      cmd.position = target.position;
      cmd.yaw = target.yaw;
      cmd.valid = true;
    }

    cmd_pub_->publish(cmd);
    health_pub_->publish(health);
  }

  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<SystemHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<Trajectory> last_traj_;
  std::optional<Clock::time_point> last_traj_time_;
  std::optional<LocalizationState> last_loc_;
  std::optional<Clock::time_point> last_loc_time_;
  double staleness_timeout_s_{0.5};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockSafety>());
  rclcpp::shutdown();
  return 0;
}
