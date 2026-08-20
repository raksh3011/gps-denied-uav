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

#include "uav_vehicle/px4_command_bridge.hpp"

#include <cmath>

namespace uav_vehicle
{

namespace
{
constexpr float kPi = 3.14159265358979323846F;

float wrapToPi(float angle)
{
  while (angle > kPi) {angle -= 2.0F * kPi;}
  while (angle < -kPi) {angle += 2.0F * kPi;}
  return angle;
}
}  // namespace

Eigen::Vector3d enuToNed(const Eigen::Vector3d & enu)
{
  return Eigen::Vector3d(enu.y(), enu.x(), -enu.z());
}

float enuYawToNed(float yaw_enu)
{
  return wrapToPi(kPi / 2.0F - yaw_enu);
}

Px4CommandBridge::Px4CommandBridge()
: Px4CommandBridge(Params())
{
}

Px4CommandBridge::Px4CommandBridge(Params params)
: params_(params)
{
}

BridgeOutput Px4CommandBridge::update(const VehicleCommandInput & in, const Px4State & px4_state)
{
  BridgeOutput out;

  if (!in.valid) {
    // Reject and hold the last safe command, per the VehicleCommand
    // contract — but "hold" means KEEP STREAMING the last known-good
    // setpoint, not stop streaming. Stopping would starve PX4's own
    // offboard-timeout watchdog and hand control to PX4's internal
    // failsafe instead of leaving the decision with our Safety module.
    if (have_last_setpoint_) {
      out.stream_setpoint = true;
      out.setpoint = last_setpoint_;
    }
    land_issued_ = false;
    rtl_issued_ = false;
    disarm_issued_ = false;
    have_held_position_ = false;
    return out;
  }

  switch (in.mode) {
    case mode::kPosition: {
        StreamedSetpoint sp;
        sp.position_control = true;
        sp.position_ned = enuToNed(in.position);
        sp.yaw_ned = enuYawToNed(in.yaw);
        out.stream_setpoint = true;
        out.setpoint = sp;
        last_setpoint_ = sp;
        have_last_setpoint_ = true;
        have_held_position_ = false;
        land_issued_ = rtl_issued_ = disarm_issued_ = false;
        break;
      }
    case mode::kVelocity: {
        StreamedSetpoint sp;
        sp.velocity_control = true;
        sp.velocity_ned = enuToNed(in.velocity);
        sp.yaw_ned = enuYawToNed(in.yaw);
        out.stream_setpoint = true;
        out.setpoint = sp;
        last_setpoint_ = sp;
        have_last_setpoint_ = true;
        have_held_position_ = false;
        land_issued_ = rtl_issued_ = disarm_issued_ = false;
        break;
      }
    case mode::kHold: {
        // Freeze at the position captured when HOLD first began (or the
        // last streamed position setpoint, if there was one) — a held
        // POSITION setpoint lets PX4 actively correct drift, unlike a
        // velocity=0 setpoint which does not.
        if (!have_held_position_) {
          if (have_last_setpoint_ && last_setpoint_.position_control) {
            held_position_ned_ = last_setpoint_.position_ned;
            held_yaw_ned_ = last_setpoint_.yaw_ned;
          } else {
            held_position_ned_ = enuToNed(in.position);
            held_yaw_ned_ = enuYawToNed(in.yaw);
          }
          have_held_position_ = true;
        }
        StreamedSetpoint sp;
        sp.position_control = true;
        sp.position_ned = held_position_ned_;
        sp.yaw_ned = held_yaw_ned_;
        out.stream_setpoint = true;
        out.setpoint = sp;
        last_setpoint_ = sp;
        have_last_setpoint_ = true;
        land_issued_ = rtl_issued_ = disarm_issued_ = false;
        break;
      }
    case mode::kLand: {
        // Keep streaming the last setpoint so PX4 stays in offboard while
        // the one-shot LAND command takes effect (LAND itself switches
        // PX4's flight mode away from Offboard; after that our stream no
        // longer matters).
        if (have_last_setpoint_) {
          out.stream_setpoint = true;
          out.setpoint = last_setpoint_;
        }
        if (!land_issued_) {
          out.command = Px4Command::kLand;
          land_issued_ = true;
        }
        rtl_issued_ = false;
        disarm_issued_ = false;
        have_held_position_ = false;
        break;
      }
    case mode::kRtl: {
        if (have_last_setpoint_) {
          out.stream_setpoint = true;
          out.setpoint = last_setpoint_;
        }
        if (!rtl_issued_) {
          out.command = Px4Command::kReturnToLaunch;
          rtl_issued_ = true;
        }
        land_issued_ = false;
        disarm_issued_ = false;
        have_held_position_ = false;
        break;
      }
    case mode::kDisarm: {
        if (!disarm_issued_) {
          out.command = Px4Command::kDisarm;
          disarm_issued_ = true;
        }
        land_issued_ = false;
        rtl_issued_ = false;
        have_held_position_ = false;
        break;
      }
    default:
      break;
  }

  // Arm + offboard-switch warm-up: PX4 requires the setpoint stream
  // running for a warm-up period before it accepts an offboard-mode
  // switch request. Only counts while actively streaming a setpoint
  // (position/velocity/hold) — never during LAND/RTL/DISARM, which are
  // deliberately taking the vehicle OUT of offboard control.
  if (out.stream_setpoint && !px4_state.offboard_active) {
    ++warmup_count_;
  } else if (!out.stream_setpoint) {
    warmup_count_ = 0;
  }

  if (out.command == Px4Command::kNone) {
    if (out.stream_setpoint && !px4_state.offboard_active &&
      warmup_count_ >= params_.offboard_warmup_ticks)
    {
      out.command = Px4Command::kSwitchToOffboard;
    } else if (out.stream_setpoint && px4_state.offboard_active && !px4_state.armed) {
      out.command = Px4Command::kArm;
    }
  }

  return out;
}

}  // namespace uav_vehicle
