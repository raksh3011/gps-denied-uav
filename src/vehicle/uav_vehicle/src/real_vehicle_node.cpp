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

// RealVehicle: the real PX4 Interface producer. Same input topic as
// MockVehicle (/safety/vehicle_command, drop-in swap per
// docs/DEVELOPMENT.md), backed by the ROS-free Px4CommandBridge (see
// px4_command_bridge.hpp and docs/VEHICLE.md).
//
// UNVERIFIED AGAINST REAL PX4 — read docs/VEHICLE.md before assuming any
// of this is confirmed working. Two specific risk areas, called out
// again at point of use below:
//  1. Topic names (`/fmu/in/...`, `/fmu/out/vehicle_status`) are exposed
//     as ROS parameters with best-guess defaults because the exact
//     names have changed across PX4 releases (notably
//     `vehicle_status` vs `vehicle_status_v1`).
//  2. VehicleStatus's arming_state/nav_state are read via px4_msgs' OWN
//     symbolic constants, never hardcoded numbers — those enum VALUES
//     have also changed across releases; the symbol names are the only
//     part guaranteed to resolve correctly for whatever px4_msgs version
//     actually got vendored.
// MAVLink command IDs (arm/disarm=400, do-set-mode=176, land=21,
// RTL=20) ARE stable across the whole MAVLink/PX4 ecosystem and are used
// as plain numbers deliberately, matching PX4's own ROS 2 examples.
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/vehicle_command.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "uav_vehicle/px4_command_bridge.hpp"

using uav_interfaces::msg::VehicleCommand;
using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
using Px4VehicleCommand = px4_msgs::msg::VehicleCommand;   // PX4's own, NOT ours — see the alias
using VehicleStatus = px4_msgs::msg::VehicleStatus;

namespace
{
// MAVLink command IDs — stable across the whole ecosystem, not
// px4_msgs-version-dependent. See the file header comment.
constexpr uint32_t kCmdComponentArmDisarm = 400;
constexpr uint32_t kCmdDoSetMode = 176;
constexpr uint32_t kCmdNavLand = 21;
constexpr uint32_t kCmdNavReturnToLaunch = 20;
constexpr float kCustomMainModeOffboard = 6.0F;   // PX4_CUSTOM_MAIN_MODE_OFFBOARD
}  // namespace

class RealVehicle : public rclcpp::Node
{
public:
  RealVehicle()
  : Node("real_vehicle")
  {
    uav_vehicle::Px4CommandBridge::Params params;
    params.offboard_warmup_ticks =
      static_cast<int>(declare_parameter<int>("offboard_warmup_ticks", 10));
    bridge_ = std::make_unique<uav_vehicle::Px4CommandBridge>(params);

    // PX4's own µXRCE-DDS QoS convention: best-effort, volatile,
    // KEEP_LAST(1) for outbound setpoints — matches PX4's ROS 2
    // offboard_control_cpp example. Not a place to reuse our internal
    // KeepLast(5) convention.
    rclcpp::QoS px4_out_qos(rclcpp::KeepLast(1));
    px4_out_qos.best_effort().durability_volatile();
    rclcpp::QoS px4_in_qos(rclcpp::KeepLast(1));
    px4_in_qos.best_effort().durability_volatile();
    rclcpp::QoS cmd_qos(rclcpp::KeepLast(5));
    cmd_qos.best_effort();

    // Topic names: best-guess defaults, exposed as parameters because
    // the exact names/versions of these topics are known to have moved
    // across PX4 releases — verify against the actual vendored px4_msgs
    // + running PX4 before trusting this, see docs/VEHICLE.md.
    const auto offboard_topic = declare_parameter<std::string>(
      "offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    const auto setpoint_topic =
      declare_parameter<std::string>("trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    const auto px4_cmd_topic =
      declare_parameter<std::string>("vehicle_command_topic", "/fmu/in/vehicle_command");
    const auto status_topic =
      declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status");

    offboard_pub_ = create_publisher<OffboardControlMode>(offboard_topic, px4_out_qos);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(setpoint_topic, px4_out_qos);
    px4_cmd_pub_ = create_publisher<Px4VehicleCommand>(px4_cmd_topic, px4_out_qos);
    status_sub_ = create_subscription<VehicleStatus>(
      status_topic, px4_in_qos,
      [this](const VehicleStatus::SharedPtr msg) {last_status_ = *msg;});

    cmd_sub_ = create_subscription<VehicleCommand>(
      "/safety/vehicle_command", cmd_qos,
      [this](const VehicleCommand::SharedPtr msg) {last_cmd_ = *msg;});

    const double rate_hz = declare_parameter<double>("rate_hz", 20.0);   // >=2Hz PX4 requirement
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RealVehicle::tick, this));
  }

private:
  void tick()
  {
    uav_vehicle::VehicleCommandInput in;
    if (last_cmd_.has_value()) {
      in.mode = last_cmd_->mode;
      in.position = Eigen::Vector3d(
        last_cmd_->position.x, last_cmd_->position.y, last_cmd_->position.z);
      in.velocity = Eigen::Vector3d(
        last_cmd_->velocity.x, last_cmd_->velocity.y, last_cmd_->velocity.z);
      in.yaw = last_cmd_->yaw;
      in.valid = last_cmd_->valid;
    }

    uav_vehicle::Px4State px4_state;
    if (last_status_.has_value()) {
      // Symbolic constants only — see the file header comment on why
      // these specific fields never use a hardcoded numeric literal.
      px4_state.armed =
        last_status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
      px4_state.offboard_active =
        last_status_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    }

    const auto out = bridge_->update(in, px4_state);
    const uint64_t now_us = static_cast<uint64_t>(this->now().nanoseconds() / 1000);

    if (out.stream_setpoint) {
      OffboardControlMode ctrl;
      ctrl.timestamp = now_us;
      ctrl.position = out.setpoint.position_control;
      ctrl.velocity = out.setpoint.velocity_control;
      offboard_pub_->publish(ctrl);

      TrajectorySetpoint sp;
      sp.timestamp = now_us;
      sp.position[0] = static_cast<float>(out.setpoint.position_ned.x());
      sp.position[1] = static_cast<float>(out.setpoint.position_ned.y());
      sp.position[2] = static_cast<float>(out.setpoint.position_ned.z());
      sp.velocity[0] = static_cast<float>(out.setpoint.velocity_ned.x());
      sp.velocity[1] = static_cast<float>(out.setpoint.velocity_ned.y());
      sp.velocity[2] = static_cast<float>(out.setpoint.velocity_ned.z());
      sp.yaw = out.setpoint.yaw_ned;
      setpoint_pub_->publish(sp);
    }

    if (out.command != uav_vehicle::Px4Command::kNone) {
      publishCommand(out.command, now_us);
    }
  }

  void publishCommand(uav_vehicle::Px4Command command, uint64_t now_us)
  {
    Px4VehicleCommand cmd;
    cmd.timestamp = now_us;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    switch (command) {
      case uav_vehicle::Px4Command::kArm:
        cmd.command = kCmdComponentArmDisarm;
        cmd.param1 = 1.0F;   // 1 = arm
        break;
      case uav_vehicle::Px4Command::kDisarm:
        cmd.command = kCmdComponentArmDisarm;
        cmd.param1 = 0.0F;   // 0 = disarm
        break;
      case uav_vehicle::Px4Command::kSwitchToOffboard:
        cmd.command = kCmdDoSetMode;
        cmd.param1 = 1.0F;   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        cmd.param2 = kCustomMainModeOffboard;
        break;
      case uav_vehicle::Px4Command::kLand:
        cmd.command = kCmdNavLand;
        break;
      case uav_vehicle::Px4Command::kReturnToLaunch:
        cmd.command = kCmdNavReturnToLaunch;
        break;
      case uav_vehicle::Px4Command::kNone:
        return;
    }
    px4_cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<Px4VehicleCommand>::SharedPtr px4_cmd_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<uav_vehicle::Px4CommandBridge> bridge_;
  std::optional<VehicleCommand> last_cmd_;
  std::optional<VehicleStatus> last_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealVehicle>());
  rclcpp::shutdown();
  return 0;
}
