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

// DemoWorld: publishes a synthetic LiDAR point cloud of a fixed pillar
// arena on /cloud_registered, standing in for FAST-LIO2's registered
// cloud so the REAL world model (and everything downstream) can be
// demonstrated without Gazebo. Demo-only — never part of a real pipeline.
//
// The arena: a field of vertical pillars between the demo start
// (-10, 0, 1.5) and MockMission's goal (10, 0, 3), spaced so the planner
// has real corridors to thread. Pillars are deliberately compact because
// downstream ObstacleSet uses a bounding-sphere approximation — a long
// wall would become one giant sphere and (with inflation margins) wall
// off the whole arena. That's a known contract limit, not a bug here.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::PointCloud2;

namespace
{
struct Pillar
{
  double x;
  double y;
  double radius;
  double height;
};

// Staggered field: no straight line from start to goal stays clear.
const std::vector<Pillar> kPillars = {
  {-6.0, 1.5, 0.5, 2.5},
  {-6.0, -3.5, 0.5, 2.5},
  {-2.0, -1.0, 0.6, 2.5},
  {-2.0, 4.0, 0.4, 2.5},
  {2.0, 1.0, 0.5, 2.5},
  {2.0, -4.0, 0.4, 2.5},
  {6.0, -1.5, 0.5, 2.5},
  {6.0, 3.0, 0.5, 2.5},
};

constexpr double kSampleStep = 0.12;   // < world model voxel size (0.25)
}  // namespace

class DemoWorld : public rclcpp::Node
{
public:
  DemoWorld()
  : Node("demo_world")
  {
    buildCloud();
    rclcpp::QoS qos(rclcpp::KeepLast(5));
    qos.best_effort();
    pub_ = create_publisher<PointCloud2>("/cloud_registered", qos);
    timer_ = create_wall_timer(100ms, std::bind(&DemoWorld::tick, this));
  }

private:
  void buildCloud()
  {
    std::vector<std::array<float, 3>> points;
    for (const auto & pillar : kPillars) {
      const double circumference = 2.0 * M_PI * pillar.radius;
      const int around = std::max(8, static_cast<int>(circumference / kSampleStep));
      const int up = std::max(2, static_cast<int>(pillar.height / kSampleStep));
      for (int a = 0; a < around; ++a) {
        const double theta = 2.0 * M_PI * a / around;
        const float px = static_cast<float>(pillar.x + pillar.radius * std::cos(theta));
        const float py = static_cast<float>(pillar.y + pillar.radius * std::sin(theta));
        for (int h = 0; h <= up; ++h) {
          points.push_back({px, py, static_cast<float>(pillar.height * h / up)});
        }
      }
    }

    cloud_.header.frame_id = "map";
    cloud_.height = 1;
    cloud_.width = static_cast<uint32_t>(points.size());
    sensor_msgs::PointCloud2Modifier modifier(cloud_);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_, "z");
    for (const auto & p : points) {
      *it_x = p[0];
      *it_y = p[1];
      *it_z = p[2];
      ++it_x;
      ++it_y;
      ++it_z;
    }
  }

  void tick()
  {
    cloud_.header.stamp = this->now();
    pub_->publish(cloud_);
  }

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  PointCloud2 cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoWorld>());
  rclcpp::shutdown();
  return 0;
}
