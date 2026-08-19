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

// MockWorldModel: subscribes to LocalizationState and publishes a fixed
// LocalMap + ObstacleSet so Planning/Safety can be developed independently
// of real LiDAR preprocessing and mapping.
#include <chrono>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/obstacle.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"

using namespace std::chrono_literals;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::LocalMap;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleSet;

class MockWorldModel : public rclcpp::Node
{
public:
  MockWorldModel()
  : Node("mock_world_model")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();

    loc_sub_ = create_subscription<LocalizationState>(
      "/localization/state", qos,
      [this](const LocalizationState::SharedPtr msg) {last_pose_ = msg->pose;});

    map_pub_ = create_publisher<LocalMap>("/world_model/local_map", qos);
    obstacle_pub_ = create_publisher<ObstacleSet>("/world_model/obstacles", qos);

    double rate_hz = declare_parameter<double>("rate_hz", 5.0);
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MockWorldModel::tick, this));
  }

private:
  void tick()
  {
    const rclcpp::Time now = this->now();

    LocalMap m;
    m.header.stamp = now;
    m.header.frame_id = "map";
    m.resolution = 0.2F;
    m.size_x = m.size_y = m.size_z = 50;
    if (last_pose_.has_value()) {
      m.origin.x = last_pose_->position.x - 5.0;
      m.origin.y = last_pose_->position.y - 5.0;
      m.origin.z = 0.0;
    }
    m.occupancy.assign(static_cast<size_t>(m.size_x) * m.size_y * m.size_z, 0);
    m.map_valid = true;
    map_pub_->publish(m);

    ObstacleSet obs_set;
    obs_set.header.stamp = now;
    obs_set.header.frame_id = "map";
    Obstacle obstacle;
    obstacle.id = 1;
    obstacle.position.x = 3.0;
    obstacle.position.y = 0.0;
    obstacle.position.z = 2.0;
    obstacle.radius = 0.5F;
    obstacle.obstacle_class = Obstacle::CLASS_STATIC;
    obs_set.obstacles.push_back(obstacle);
    obstacle_pub_->publish(obs_set);
  }

  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Publisher<LocalMap>::SharedPtr map_pub_;
  rclcpp::Publisher<ObstacleSet>::SharedPtr obstacle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::Pose> last_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockWorldModel>());
  rclcpp::shutdown();
  return 0;
}
