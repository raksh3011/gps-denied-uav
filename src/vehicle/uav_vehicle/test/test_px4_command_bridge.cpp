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

#include <gtest/gtest.h>

#include "uav_vehicle/px4_command_bridge.hpp"

using uav_vehicle::BridgeOutput;
using uav_vehicle::Px4Command;
using uav_vehicle::Px4CommandBridge;
using uav_vehicle::Px4State;
using uav_vehicle::VehicleCommandInput;
using uav_vehicle::enuToNed;
using uav_vehicle::enuYawToNed;

namespace
{
constexpr float kPi = 3.14159265358979323846F;
}  // namespace

TEST(Px4FrameConvert, EnuToNedSwapsAndNegatesZ) {
  const Eigen::Vector3d ned = enuToNed(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_DOUBLE_EQ(ned.x(), 2.0);
  EXPECT_DOUBLE_EQ(ned.y(), 1.0);
  EXPECT_DOUBLE_EQ(ned.z(), -3.0);
}

TEST(Px4FrameConvert, YawZeroEnuIsQuarterTurnNed) {
  // ENU yaw 0 = facing +X (east). NED yaw 0 = facing +X (north).
  // East in NED-heading terms is +90 deg (pi/2).
  EXPECT_NEAR(enuYawToNed(0.0F), kPi / 2.0F, 1e-5);
}

TEST(Px4FrameConvert, YawWrapsToPiRange) {
  const float wrapped = enuYawToNed(-kPi);
  EXPECT_LE(wrapped, kPi);
  EXPECT_GE(wrapped, -kPi);
}

TEST(Px4CommandBridge, PositionModeStreamsConvertedSetpoint) {
  Px4CommandBridge bridge;
  VehicleCommandInput in;
  in.mode = uav_vehicle::mode::kPosition;
  in.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  in.yaw = 0.0F;
  in.valid = true;

  const auto out = bridge.update(in, Px4State{});
  EXPECT_TRUE(out.stream_setpoint);
  EXPECT_TRUE(out.setpoint.position_control);
  EXPECT_FALSE(out.setpoint.velocity_control);
  EXPECT_EQ(out.setpoint.position_ned, enuToNed(in.position));
}

TEST(Px4CommandBridge, VelocityModeStreamsConvertedVelocity) {
  Px4CommandBridge bridge;
  VehicleCommandInput in;
  in.mode = uav_vehicle::mode::kVelocity;
  in.velocity = Eigen::Vector3d(0.5, 0.0, 0.0);
  in.valid = true;

  const auto out = bridge.update(in, Px4State{});
  EXPECT_TRUE(out.stream_setpoint);
  EXPECT_TRUE(out.setpoint.velocity_control);
  EXPECT_FALSE(out.setpoint.position_control);
  EXPECT_EQ(out.setpoint.velocity_ned, enuToNed(in.velocity));
}

TEST(Px4CommandBridge, InvalidCommandKeepsStreamingLastSetpoint) {
  Px4CommandBridge bridge;
  VehicleCommandInput good;
  good.mode = uav_vehicle::mode::kPosition;
  good.position = Eigen::Vector3d(4.0, 5.0, 6.0);
  good.valid = true;
  const auto first = bridge.update(good, Px4State{});
  ASSERT_TRUE(first.stream_setpoint);

  VehicleCommandInput bad;
  bad.valid = false;
  const auto out = bridge.update(bad, Px4State{});
  EXPECT_TRUE(out.stream_setpoint) << "must keep streaming, not go silent, on an invalid command";
  EXPECT_EQ(out.setpoint.position_ned, enuToNed(good.position));
  EXPECT_EQ(out.command, Px4Command::kNone) << "no new arm/offboard request while faulted";
}

TEST(Px4CommandBridge, InvalidCommandWithNoPriorSetpointStreamsNothing) {
  Px4CommandBridge bridge;
  VehicleCommandInput bad;
  bad.valid = false;
  const auto out = bridge.update(bad, Px4State{});
  EXPECT_FALSE(out.stream_setpoint);
}

TEST(Px4CommandBridge, HoldFreezesAtFirstHeldPosition) {
  Px4CommandBridge bridge;
  VehicleCommandInput pos;
  pos.mode = uav_vehicle::mode::kPosition;
  pos.position = Eigen::Vector3d(1.0, 1.0, 1.0);
  pos.valid = true;
  bridge.update(pos, Px4State{});

  VehicleCommandInput hold;
  hold.mode = uav_vehicle::mode::kHold;
  hold.valid = true;
  const auto first_hold = bridge.update(hold, Px4State{});
  ASSERT_TRUE(first_hold.stream_setpoint);
  EXPECT_EQ(first_hold.setpoint.position_ned, enuToNed(pos.position));

  // A second HOLD tick (even with different position field content, e.g.
  // stale/irrelevant) must stay frozen at the FIRST held position.
  hold.position = Eigen::Vector3d(99.0, 99.0, 99.0);
  const auto second_hold = bridge.update(hold, Px4State{});
  EXPECT_EQ(second_hold.setpoint.position_ned, enuToNed(pos.position));
}

TEST(Px4CommandBridge, LandIsIssuedOnceNotEveryTick) {
  Px4CommandBridge bridge;
  VehicleCommandInput land;
  land.mode = uav_vehicle::mode::kLand;
  land.valid = true;

  const auto first = bridge.update(land, Px4State{});
  EXPECT_EQ(first.command, Px4Command::kLand);

  const auto second = bridge.update(land, Px4State{});
  EXPECT_EQ(second.command, Px4Command::kNone) << "LAND must be one-shot, not repeated every tick";
}

TEST(Px4CommandBridge, LandReissuesAfterModeChangesAwayAndBack) {
  Px4CommandBridge bridge;
  VehicleCommandInput land;
  land.mode = uav_vehicle::mode::kLand;
  land.valid = true;
  bridge.update(land, Px4State{});   // issues once

  VehicleCommandInput pos;
  pos.mode = uav_vehicle::mode::kPosition;
  pos.valid = true;
  bridge.update(pos, Px4State{});   // switches away from LAND

  const auto reissued = bridge.update(land, Px4State{});
  EXPECT_EQ(reissued.command, Px4Command::kLand);
}

TEST(Px4CommandBridge, RtlIsIssuedOnce) {
  Px4CommandBridge bridge;
  VehicleCommandInput rtl;
  rtl.mode = uav_vehicle::mode::kRtl;
  rtl.valid = true;

  EXPECT_EQ(bridge.update(rtl, Px4State{}).command, Px4Command::kReturnToLaunch);
  EXPECT_EQ(bridge.update(rtl, Px4State{}).command, Px4Command::kNone);
}

TEST(Px4CommandBridge, DisarmIsIssuedOnceAndDoesNotStream) {
  Px4CommandBridge bridge;
  VehicleCommandInput disarm;
  disarm.mode = uav_vehicle::mode::kDisarm;
  disarm.valid = true;

  const auto out = bridge.update(disarm, Px4State{});
  EXPECT_EQ(out.command, Px4Command::kDisarm);
  EXPECT_FALSE(out.stream_setpoint);
  EXPECT_EQ(bridge.update(disarm, Px4State{}).command, Px4Command::kNone);
}

TEST(Px4CommandBridge, SwitchToOffboardRequestedAfterWarmup) {
  Px4CommandBridge::Params params;
  params.offboard_warmup_ticks = 3;
  Px4CommandBridge bridge(params);

  VehicleCommandInput pos;
  pos.mode = uav_vehicle::mode::kPosition;
  pos.valid = true;

  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(bridge.update(pos, Px4State{}).command, Px4Command::kNone)
      << "must not request offboard before warm-up completes";
  }
  EXPECT_EQ(bridge.update(pos, Px4State{}).command, Px4Command::kSwitchToOffboard);
}

TEST(Px4CommandBridge, ArmRequestedOnceOffboardActiveButNotArmed) {
  Px4CommandBridge::Params params;
  params.offboard_warmup_ticks = 1;
  Px4CommandBridge bridge(params);

  VehicleCommandInput pos;
  pos.mode = uav_vehicle::mode::kPosition;
  pos.valid = true;

  bridge.update(pos, Px4State{false, false});   // warm-up tick
  const auto switch_out = bridge.update(pos, Px4State{false, false});
  EXPECT_EQ(switch_out.command, Px4Command::kSwitchToOffboard);

  // PX4 reports offboard now active, still disarmed.
  const auto arm_out = bridge.update(pos, Px4State{false, true});
  EXPECT_EQ(arm_out.command, Px4Command::kArm);

  // Once armed, no more arm/offboard requests.
  const auto steady_out = bridge.update(pos, Px4State{true, true});
  EXPECT_EQ(steady_out.command, Px4Command::kNone);
}

TEST(Px4CommandBridge, NoOffboardRequestDuringLand) {
  Px4CommandBridge::Params params;
  params.offboard_warmup_ticks = 1;
  Px4CommandBridge bridge(params);

  VehicleCommandInput land;
  land.mode = uav_vehicle::mode::kLand;
  land.valid = true;

  // Even across many ticks, LAND must never accumulate warm-up progress
  // toward an offboard-switch request — it's deliberately leaving
  // offboard control, not entering it.
  for (int i = 0; i < 5; ++i) {
    const auto out = bridge.update(land, Px4State{});
    EXPECT_NE(out.command, Px4Command::kSwitchToOffboard);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
