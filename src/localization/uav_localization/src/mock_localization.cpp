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

// MockLocalization: publishes a plausible LocalizationState at a fixed rate
// so downstream modules (World Model, Planning, Safety) can be developed and
// tested without a real LiDAR-Inertial Odometry pipeline.
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/localization_state.hpp"

using namespace std::chrono_literals;
using uav_interfaces::msg::LocalizationState;

class MockLocalization : public rclcpp::Node
{
public:
  MockLocalization()
  : Node("mock_localization"), t0_(this->now())
  {
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    pub_ = create_publisher<LocalizationState>("/localization/state", qos);

    double rate_hz = declare_parameter<double>("rate_hz", 50.0);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockLocalization::tick, this));
  }

private:
  void tick()
  {
    const rclcpp::Time now = this->now();
    const double t = (now - t0_).seconds();

    LocalizationState msg;
    msg.header.stamp = now;
    msg.header.frame_id = "map";

    // Gentle circular trajectory as a stand-in for real odometry.
    constexpr double radius = 5.0;
    constexpr double omega = 0.1;
    msg.pose.position.x = radius * std::cos(omega * t);
    msg.pose.position.y = radius * std::sin(omega * t);
    msg.pose.position.z = 2.0;
    msg.pose.orientation.w = 1.0;

    msg.twist.linear.x = -radius * omega * std::sin(omega * t);
    msg.twist.linear.y = radius * omega * std::cos(omega * t);

    for (size_t i = 0; i < msg.pose_covariance.size(); ++i) {
      msg.pose_covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
      msg.twist_covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }

    msg.confidence = 0.95F;
    msg.localization_ok = true;
    msg.status = LocalizationState::STATUS_NOMINAL;

    pub_->publish(msg);
  }

  rclcpp::Publisher<LocalizationState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time t0_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockLocalization>());
  rclcpp::shutdown();
  return 0;
}
