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

// VizBridge: converts the stack's custom uav_interfaces messages into
// standard RViz-displayable types. Pure visualization — publishes nothing
// any real module consumes, so it can be added/removed freely.
//   /localization/state    -> /viz/pose (PoseStamped) + /viz/track (Path, capped trail)
//   /planning/trajectory   -> /viz/planned_path (Path)
//   /world_model/obstacles -> /viz/obstacles (MarkerArray, red spheres;
//                             dynamic obstacles orange)
//   /world_model/local_map -> /viz/occupied_voxels (Marker CUBE_LIST)
//   /mission/current       -> /viz/goal (Marker, green sphere)
// Publishers are RELIABLE on purpose: RViz display subscriptions default
// to reliable QoS, which never matches our internal best-effort topics.
#include <array>
#include <chrono>
#include <deque>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/mission.hpp"
#include "uav_interfaces/msg/obstacle.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"
#include "uav_interfaces/msg/trajectory.hpp"

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::LocalMap;
using uav_interfaces::msg::Mission;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleSet;
using uav_interfaces::msg::Trajectory;

namespace
{
constexpr size_t kMaxTrailPoses = 600;   // ~60s of trail at 10 Hz
}  // namespace

class VizBridge : public rclcpp::Node
{
public:
  VizBridge()
  : Node("viz_bridge")
  {
    rclcpp::QoS sub_qos(rclcpp::KeepLast(5));
    sub_qos.best_effort();
    rclcpp::QoS mission_qos(rclcpp::KeepLast(1));
    mission_qos.reliable().transient_local();
    rclcpp::QoS pub_qos(rclcpp::KeepLast(5));
    pub_qos.reliable();

    pose_pub_ = create_publisher<PoseStamped>("/viz/pose", pub_qos);
    drone_pub_ = create_publisher<MarkerArray>("/viz/drone", pub_qos);
    track_pub_ = create_publisher<Path>("/viz/track", pub_qos);
    path_pub_ = create_publisher<Path>("/viz/planned_path", pub_qos);
    obstacles_pub_ = create_publisher<MarkerArray>("/viz/obstacles", pub_qos);
    voxels_pub_ = create_publisher<Marker>("/viz/occupied_voxels", pub_qos);
    goal_pub_ = create_publisher<Marker>("/viz/goal", pub_qos);

    loc_sub_ = create_subscription<LocalizationState>(
      "/localization/state", sub_qos,
      std::bind(&VizBridge::onLocalization, this, std::placeholders::_1));
    traj_sub_ = create_subscription<Trajectory>(
      "/planning/trajectory", sub_qos,
      std::bind(&VizBridge::onTrajectory, this, std::placeholders::_1));
    obstacles_sub_ = create_subscription<ObstacleSet>(
      "/world_model/obstacles", sub_qos,
      std::bind(&VizBridge::onObstacles, this, std::placeholders::_1));
    map_sub_ = create_subscription<LocalMap>(
      "/world_model/local_map", sub_qos,
      std::bind(&VizBridge::onMap, this, std::placeholders::_1));
    mission_sub_ = create_subscription<Mission>(
      "/mission/current", mission_qos,
      std::bind(&VizBridge::onMission, this, std::placeholders::_1));
  }

private:
  void onLocalization(const LocalizationState::SharedPtr msg)
  {
    PoseStamped pose;
    pose.header = msg->header;
    pose.header.frame_id = "map";
    pose.pose = msg->pose;
    pose_pub_->publish(pose);
    publishDroneBody(pose);

    trail_.push_back(pose);
    if (trail_.size() > kMaxTrailPoses) {trail_.pop_front();}
    // Publish the trail at a decimated rate — every localization tick
    // would be 50 Hz of a growing message for no visual benefit.
    if (++trail_ticks_ % 5 != 0) {return;}
    Path track;
    track.header = pose.header;
    track.poses.assign(trail_.begin(), trail_.end());
    track_pub_->publish(track);
  }

  // A simple quadcopter silhouette: dark body box + 4 rotor disks.
  void publishDroneBody(const PoseStamped & pose)
  {
    MarkerArray drone;
    Marker body;
    body.header = pose.header;
    body.ns = "drone";
    body.id = 0;
    body.type = Marker::CUBE;
    body.action = Marker::ADD;
    body.pose = pose.pose;
    body.scale.x = 0.45;
    body.scale.y = 0.45;
    body.scale.z = 0.14;
    body.color.r = 0.15F;
    body.color.g = 0.15F;
    body.color.b = 0.18F;
    body.color.a = 1.0F;
    drone.markers.push_back(body);

    const double arm = 0.34;
    const std::array<std::array<double, 2>, 4> offsets = {{
      {arm, arm}, {arm, -arm}, {-arm, arm}, {-arm, -arm}
    }};
    for (size_t i = 0; i < offsets.size(); ++i) {
      Marker rotor;
      rotor.header = pose.header;
      rotor.ns = "drone";
      rotor.id = static_cast<int>(i) + 1;
      rotor.type = Marker::CYLINDER;
      rotor.action = Marker::ADD;
      rotor.pose = pose.pose;
      rotor.pose.position.x += offsets[i][0];
      rotor.pose.position.y += offsets[i][1];
      rotor.pose.position.z += 0.09;
      rotor.scale.x = 0.30;
      rotor.scale.y = 0.30;
      rotor.scale.z = 0.03;
      const bool front = offsets[i][0] > 0.0;
      rotor.color.r = front ? 0.95F : 0.35F;
      rotor.color.g = front ? 0.45F : 0.35F;
      rotor.color.b = front ? 0.1F : 0.4F;
      rotor.color.a = 0.95F;
      drone.markers.push_back(rotor);
    }
    drone_pub_->publish(drone);
  }

  void onTrajectory(const Trajectory::SharedPtr msg)
  {
    if (!msg->valid) {return;}
    Path path;
    path.header = msg->header;
    path.header.frame_id = "map";
    path.poses.reserve(msg->points.size());
    for (const auto & point : msg->points) {
      PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point.position;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }

  void onObstacles(const ObstacleSet::SharedPtr msg)
  {
    MarkerArray markers;
    Marker clear;
    clear.header.frame_id = "map";
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (const auto & obstacle : msg->obstacles) {
      Marker m;
      m.header = msg->header;
      m.header.frame_id = "map";
      m.ns = "obstacles";
      m.id = static_cast<int>(obstacle.id);
      m.type = Marker::SPHERE;
      m.action = Marker::ADD;
      m.pose.position = obstacle.position;
      m.pose.orientation.w = 1.0;
      const double d = 2.0 * obstacle.radius;
      m.scale.x = m.scale.y = m.scale.z = d;
      const bool dynamic = obstacle.obstacle_class == Obstacle::CLASS_DYNAMIC;
      m.color.r = 1.0F;
      m.color.g = dynamic ? 0.55F : 0.15F;
      m.color.b = 0.1F;
      m.color.a = 0.75F;
      markers.markers.push_back(m);
    }
    obstacles_pub_->publish(markers);
  }

  void onMap(const LocalMap::SharedPtr msg)
  {
    if (!msg->map_valid) {return;}
    Marker cubes;
    cubes.header = msg->header;
    cubes.header.frame_id = "map";
    cubes.ns = "occupied";
    cubes.id = 0;
    cubes.type = Marker::CUBE_LIST;
    cubes.action = Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    cubes.scale.x = cubes.scale.y = cubes.scale.z = msg->resolution;
    cubes.color.r = 0.3F;
    cubes.color.g = 0.5F;
    cubes.color.b = 0.9F;
    cubes.color.a = 0.6F;

    const auto sx = msg->size_x;
    const auto sy = msg->size_y;
    for (size_t i = 0; i < msg->occupancy.size(); ++i) {
      if (msg->occupancy[i] == 0) {continue;}
      const uint32_t x = static_cast<uint32_t>(i % sx);
      const uint32_t y = static_cast<uint32_t>((i / sx) % sy);
      const uint32_t z = static_cast<uint32_t>(i / (static_cast<size_t>(sx) * sy));
      geometry_msgs::msg::Point p;
      p.x = msg->origin.x + (x + 0.5) * msg->resolution;
      p.y = msg->origin.y + (y + 0.5) * msg->resolution;
      p.z = msg->origin.z + (z + 0.5) * msg->resolution;
      cubes.points.push_back(p);
    }
    voxels_pub_->publish(cubes);
  }

  void onMission(const Mission::SharedPtr msg)
  {
    if (msg->waypoints.empty()) {return;}
    Marker goal;
    goal.header = msg->header;
    goal.header.frame_id = "map";
    goal.ns = "goal";
    goal.id = 0;
    goal.type = Marker::SPHERE;
    goal.action = Marker::ADD;
    goal.pose.position = msg->waypoints.front().position;
    goal.pose.orientation.w = 1.0;
    goal.scale.x = goal.scale.y = goal.scale.z = 0.6;
    goal.color.r = 0.1F;
    goal.color.g = 0.9F;
    goal.color.b = 0.2F;
    goal.color.a = 0.9F;
    goal_pub_->publish(goal);
  }

  rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr drone_pub_;
  rclcpp::Publisher<Path>::SharedPtr track_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<Marker>::SharedPtr voxels_pub_;
  rclcpp::Publisher<Marker>::SharedPtr goal_pub_;

  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<ObstacleSet>::SharedPtr obstacles_sub_;
  rclcpp::Subscription<LocalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<Mission>::SharedPtr mission_sub_;

  std::deque<PoseStamped> trail_;
  size_t trail_ticks_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VizBridge>());
  rclcpp::shutdown();
  return 0;
}
