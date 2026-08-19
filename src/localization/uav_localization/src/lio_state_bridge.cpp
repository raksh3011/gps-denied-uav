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

// LioStateBridge: the real Localization producer. Subscribes to a
// LiDAR-Inertial Odometry backend's nav_msgs/Odometry output (FAST-LIO2's
// "/Odometry" topic by default) and republishes it as the frozen
// LocalizationState contract every other module already builds against.
//
// This node does NOT run any estimation itself — FAST-LIO2 (or whichever
// LIO backend is launched alongside it, see launch/real_localization.launch.py)
// owns the actual state estimate. This node's only job is the adapter
// boundary: convert Odometry -> LocalizationState, and turn silence/staleness
// from the backend into an honest localization_ok=false rather than letting
// stale data look live.
//
// Confidence/status heuristic (v1, deliberately simple — revisit once real
// flight data exists):
//   - STATUS_NOMINAL, confidence 0.9, localization_ok=true   while odometry
//     arrives within `staleness_timeout_s` of now.
//   - STATUS_DEGRADED, confidence 0.3, localization_ok=false while stale for
//     longer than that but under `lost_timeout_s`.
//   - STATUS_LOST, confidence 0.0, localization_ok=false once stale beyond
//     `lost_timeout_s` — e.g. the LIO backend has died or lost tracking.
#include <chrono>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "uav_interfaces/msg/localization_state.hpp"

using nav_msgs::msg::Odometry;
using uav_interfaces::msg::LocalizationState;
using Clock = std::chrono::steady_clock;

class LioStateBridge : public rclcpp::Node
{
public:
  LioStateBridge()
  : Node("lio_state_bridge")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();

    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/Odometry");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    staleness_timeout_s_ = declare_parameter<double>("staleness_timeout_s", 0.3);
    lost_timeout_s_ = declare_parameter<double>("lost_timeout_s", 1.5);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);

    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, qos,
      [this](const Odometry::SharedPtr msg) {
        last_odom_ = *msg;
        last_odom_time_ = Clock::now();
      });

    pub_ = create_publisher<LocalizationState>("/localization/state", qos);

    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LioStateBridge::tick, this));

    RCLCPP_INFO(
      get_logger(), "lio_state_bridge: relaying %s -> /localization/state",
      odom_topic_.c_str());
  }

private:
  void tick()
  {
    LocalizationState out;
    out.header.frame_id = map_frame_;

    if (!last_odom_.has_value()) {
      out.header.stamp = this->now();
      out.localization_ok = false;
      out.confidence = 0.0F;
      out.status = LocalizationState::STATUS_LOST;
      pub_->publish(out);
      return;
    }

    const double age = std::chrono::duration<double>(Clock::now() - *last_odom_time_).count();

    out.header.stamp = last_odom_->header.stamp;
    out.pose = last_odom_->pose.pose;
    out.twist = last_odom_->twist.twist;
    out.pose_covariance = last_odom_->pose.covariance;
    out.twist_covariance = last_odom_->twist.covariance;

    if (age <= staleness_timeout_s_) {
      out.status = LocalizationState::STATUS_NOMINAL;
      out.confidence = 0.9F;
      out.localization_ok = true;
    } else if (age <= lost_timeout_s_) {
      out.status = LocalizationState::STATUS_DEGRADED;
      out.confidence = 0.3F;
      out.localization_ok = false;
    } else {
      out.status = LocalizationState::STATUS_LOST;
      out.confidence = 0.0F;
      out.localization_ok = false;
    }

    pub_->publish(out);
  }

  std::string odom_topic_;
  std::string map_frame_;
  double staleness_timeout_s_{0.3};
  double lost_timeout_s_{1.5};
  double publish_rate_hz_{100.0};

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<LocalizationState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<Odometry> last_odom_;
  std::optional<Clock::time_point> last_odom_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioStateBridge>());
  rclcpp::shutdown();
  return 0;
}
