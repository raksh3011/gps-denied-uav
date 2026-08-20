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

// DemoFlyer: a demo-only stand-in for Localization+Vehicle that CLOSES
// THE LOOP: instead of MockLocalization's scripted circle, it moves the
// simulated vehicle along the planner's own /planning/trajectory with a
// simple pure-pursuit follower, and publishes the resulting pose as
// LocalizationState. What you see in RViz is therefore the real planner
// flying its own plan through the real world model's map — not a script.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/trajectory.hpp"

using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::Trajectory;

class DemoFlyer : public rclcpp::Node
{
public:
  DemoFlyer()
  : Node("demo_flyer")
  {
    x_ = declare_parameter<double>("start_x", -10.0);
    y_ = declare_parameter<double>("start_y", 0.0);
    z_ = declare_parameter<double>("start_z", 1.5);
    speed_ = declare_parameter<double>("speed_mps", 2.0);
    lookahead_ = declare_parameter<double>("lookahead_m", 1.2);

    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    traj_sub_ = create_subscription<Trajectory>(
      "/planning/trajectory", qos,
      [this](const Trajectory::SharedPtr msg) {if (msg->valid) {traj_ = *msg;}});
    pub_ = create_publisher<LocalizationState>("/localization/state", qos);

    const double rate_hz = declare_parameter<double>("rate_hz", 30.0);
    dt_ = 1.0 / rate_hz;
    auto period = std::chrono::duration<double>(dt_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DemoFlyer::tick, this));
  }

private:
  void tick()
  {
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

    if (traj_.has_value() && !traj_->points.empty()) {
      // Pure pursuit: chase the first trajectory point at least
      // `lookahead_` ahead of the vehicle; the final point if none is.
      const auto & points = traj_->points;
      size_t target = points.size() - 1;
      for (size_t i = 0; i < points.size(); ++i) {
        const double dx = points[i].position.x - x_;
        const double dy = points[i].position.y - y_;
        const double dz = points[i].position.z - z_;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) >= lookahead_) {
          target = i;
          break;
        }
      }
      const double dx = points[target].position.x - x_;
      const double dy = points[target].position.y - y_;
      const double dz = points[target].position.z - z_;
      const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist > 0.05) {
        const double step = std::min(speed_ * dt_, dist);
        vx = dx / dist * speed_;
        vy = dy / dist * speed_;
        vz = dz / dist * speed_;
        x_ += dx / dist * step;
        y_ += dy / dist * step;
        z_ += dz / dist * step;
      }
    }

    LocalizationState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.pose.position.x = x_;
    msg.pose.position.y = y_;
    msg.pose.position.z = z_;
    msg.pose.orientation.w = 1.0;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
    for (size_t i = 0; i < msg.pose_covariance.size(); ++i) {
      msg.pose_covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
      msg.twist_covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }
    msg.confidence = 0.95F;
    msg.localization_ok = true;
    msg.status = LocalizationState::STATUS_NOMINAL;
    pub_->publish(msg);
  }

  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<LocalizationState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<Trajectory> traj_;
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double speed_{2.0};
  double lookahead_{1.2};
  double dt_{1.0 / 30.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoFlyer>());
  rclcpp::shutdown();
  return 0;
}
