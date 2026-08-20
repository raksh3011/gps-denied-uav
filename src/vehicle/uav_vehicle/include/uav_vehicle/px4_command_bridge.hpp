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

// Px4CommandBridge: the real PX4 Interface decision logic. ROS-free (same
// pattern as uav_planning_core/uav_world_model_core/uav_safety_core) so
// it's unit-testable without a ROS runtime and, notably, without the
// vendored px4_msgs dependency at all — real_vehicle_node is the only
// place this gets wired to actual PX4 uORB topics over uXRCE-DDS. See
// docs/VEHICLE.md for the full design rationale and, importantly, what
// here is verified-by-test vs. still unverified against real PX4.
//
// Frame: PX4 setpoints are NED (North-East-Down); our own contract
// (VehicleCommand, per docs/CONVENTIONS.md) is ENU (East-North-Up). The
// conversion used (enuToNed / enuYawToNed) is the standard one from
// PX4's own ROS 2 offboard_control_cpp example: x_ned=y_enu, y_ned=x_enu,
// z_ned=-z_enu, yaw_ned = pi/2 - yaw_enu (wrapped to [-pi, pi]).
#ifndef UAV_VEHICLE__PX4_COMMAND_BRIDGE_HPP_
#define UAV_VEHICLE__PX4_COMMAND_BRIDGE_HPP_

#include <Eigen/Core>

#include <cstdint>

namespace uav_vehicle
{

Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu);
float enuYawToNed(float yaw_enu);

// Mirrors uav_interfaces::msg::VehicleCommand's fields exactly (plain
// types, no ROS message dependency in this ROS-free core).
struct VehicleCommandInput
{
  uint8_t mode{0};   // VehicleCommand::MODE_* — see mode constants below
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};   // ENU, meters
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};   // ENU, m/s
  float yaw{0.0F};   // ENU, radians
  bool valid{false};
};

// Mirrors the frozen VehicleCommand.msg mode enum — kept as plain
// constants here, same convention used elsewhere in this codebase
// (DStarLitePlanner::riskBandFor, SafetyMonitor), so this core stays
// message-type-agnostic.
namespace mode
{
constexpr uint8_t kPosition = 0;
constexpr uint8_t kVelocity = 1;
constexpr uint8_t kLand = 2;
constexpr uint8_t kRtl = 3;
constexpr uint8_t kHold = 4;
constexpr uint8_t kDisarm = 5;
}  // namespace mode

// One-shot PX4 uORB VehicleCommand to issue this tick, if any — arm,
// offboard mode switch, land, RTL, disarm are all single commands, not
// something to stream every tick like a setpoint.
enum class Px4Command
{
  kNone,
  kArm,
  kDisarm,
  kSwitchToOffboard,
  kLand,
  kReturnToLaunch,
};

// What to stream this tick, if anything — real_vehicle_node publishes
// this as OffboardControlMode + TrajectorySetpoint every tick while
// streaming is true. NED, ready for direct assignment to px4_msgs fields.
struct StreamedSetpoint
{
  bool position_control{false};   // sets OffboardControlMode.position
  bool velocity_control{false};   // sets OffboardControlMode.velocity
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  float yaw_ned{0.0F};
};

struct BridgeOutput
{
  bool stream_setpoint{false};
  StreamedSetpoint setpoint;
  Px4Command command{Px4Command::kNone};
};

// px4_armed / px4_offboard_active: PX4's own reported state (from
// VehicleStatus), fed back in so the bridge doesn't re-issue arm/offboard
// switch commands PX4 has already acted on.
struct Px4State
{
  bool armed{false};
  bool offboard_active{false};
};

class Px4CommandBridge
{
public:
  struct Params
  {
    // PX4 requires the setpoint stream running for a warm-up period
    // before it will accept an offboard-mode switch request — this is
    // the tick count (at whatever rate the caller ticks, matching
    // OffboardControlMode's own >=2Hz requirement) to stream before
    // requesting arm+offboard. 10 matches PX4's own ROS 2
    // offboard_control_cpp example.
    int offboard_warmup_ticks{10};
  };

  Px4CommandBridge();
  explicit Px4CommandBridge(Params params);

  // Call once per tick with the latest VehicleCommand and PX4's current
  // reported state.
  BridgeOutput update(const VehicleCommandInput & in, const Px4State & px4_state);

private:
  Params params_;
  int warmup_count_{0};
  bool land_issued_{false};
  bool rtl_issued_{false};
  bool disarm_issued_{false};
  bool have_held_position_{false};
  Eigen::Vector3d held_position_ned_{Eigen::Vector3d::Zero()};
  float held_yaw_ned_{0.0F};
  StreamedSetpoint last_setpoint_;
  bool have_last_setpoint_{false};
};

}  // namespace uav_vehicle

#endif  // UAV_VEHICLE__PX4_COMMAND_BRIDGE_HPP_
