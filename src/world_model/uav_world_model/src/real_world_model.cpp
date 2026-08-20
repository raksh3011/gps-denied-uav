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

// RealWorldModel: the real LocalMap/ObstacleSet producer. Same topics/QoS
// as MockWorldModel (drop-in swap per docs/DEVELOPMENT.md), backed by
// VoxelMapper + clusterOccupied/ObstacleTracker (see those headers and
// docs/WORLD_MODEL.md).
//
// Inputs:
//  - `cloud_topic` (default /cloud_registered): FAST-LIO2's registered
//    point cloud. ASSUMPTION (documented in docs/WORLD_MODEL.md): this
//    cloud is expressed in the same frame we call "map" — the LIO odom
//    frame that lio_state_bridge also passes through as LocalizationState.
//    No TF lookup is performed.
//  - /localization/state: vehicle pose, drives window re-centering, and
//    map_valid gating.
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "uav_interfaces/msg/localization_state.hpp"
#include "uav_interfaces/msg/local_map.hpp"
#include "uav_interfaces/msg/obstacle.hpp"
#include "uav_interfaces/msg/obstacle_set.hpp"

#include "uav_world_model/obstacle_tracker.hpp"
#include "uav_world_model/voxel_mapper.hpp"

using sensor_msgs::msg::PointCloud2;
using uav_interfaces::msg::LocalizationState;
using uav_interfaces::msg::LocalMap;
using uav_interfaces::msg::Obstacle;
using uav_interfaces::msg::ObstacleSet;

class RealWorldModel : public rclcpp::Node
{
public:
  RealWorldModel()
  : Node("real_world_model"),
    tracker_(
      declare_parameter<double>("track_gate_m", 1.0),
      declare_parameter<double>("dynamic_speed_mps", 0.3),
      static_cast<int>(declare_parameter<int>("max_missed_frames", 3)))
  {
    uav_world_model::VoxelMapperParams params;
    params.resolution = declare_parameter<double>("resolution", 0.2);
    params.size_x = static_cast<int>(declare_parameter<int>("size_x", 50));
    params.size_y = static_cast<int>(declare_parameter<int>("size_y", 50));
    params.size_z = static_cast<int>(declare_parameter<int>("size_z", 30));
    params.min_hits = static_cast<int>(declare_parameter<int>("min_hits", 2));
    params.max_hits = static_cast<int>(declare_parameter<int>("max_hits", 10));
    params.decay_per_call = static_cast<int>(declare_parameter<int>("decay_per_call", 1));
    params.recenter_threshold_m = declare_parameter<double>("recenter_threshold_m", 2.0);
    mapper_ = std::make_unique<uav_world_model::VoxelMapper>(params);

    point_stride_ = static_cast<int>(declare_parameter<int>("point_stride", 2));
    decay_every_n_ticks_ = static_cast<int>(declare_parameter<int>("decay_every_n_ticks", 10));
    min_cluster_voxels_ = static_cast<int>(declare_parameter<int>("min_cluster_voxels", 3));
    // 0 = disabled (one sphere per connected component, no slicing). See
    // clusterOccupied's header comment for why this matters for tall/thin
    // obstacles (poles, trees, pillars).
    max_cluster_height_m_ = declare_parameter<double>("max_cluster_height_m", 1.2);

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
    sensor_qos.best_effort();

    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
    cloud_sub_ = create_subscription<PointCloud2>(
      cloud_topic, sensor_qos,
      std::bind(&RealWorldModel::onCloud, this, std::placeholders::_1));
    loc_sub_ = create_subscription<LocalizationState>(
      "/localization/state", sensor_qos,
      [this](const LocalizationState::SharedPtr msg) {loc_ = *msg;});

    map_pub_ = create_publisher<LocalMap>("/world_model/local_map", sensor_qos);
    obstacle_pub_ = create_publisher<ObstacleSet>("/world_model/obstacles", sensor_qos);

    const double rate_hz = declare_parameter<double>("rate_hz", 5.0);
    tick_period_s_ = 1.0 / rate_hz;
    auto period = std::chrono::duration<double>(tick_period_s_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RealWorldModel::tick, this));
  }

private:
  void onCloud(const PointCloud2::SharedPtr msg)
  {
    // Parse x/y/z and stash for the next tick; strided to bound work on
    // dense clouds. Non-finite points (LiDAR no-return) are dropped.
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    int i = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++i) {
      if (i % point_stride_ != 0) {continue;}
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
      pending_points_.emplace_back(x, y, z);
    }
  }

  void tick()
  {
    const rclcpp::Time now = this->now();

    if (loc_.has_value() && loc_->localization_ok) {
      const auto & p = loc_->pose.position;
      mapper_->maybeRecenter(Eigen::Vector3d(p.x, p.y, p.z));
    }

    if (!pending_points_.empty()) {
      mapper_->integratePoints(pending_points_);
      pending_points_.clear();
    }
    if (++ticks_since_decay_ >= decay_every_n_ticks_) {
      mapper_->decay();
      ticks_since_decay_ = 0;
    }

    LocalMap map;
    map.header.stamp = now;
    map.header.frame_id = "map";
    map.resolution = static_cast<float>(mapper_->resolution());
    map.origin.x = mapper_->origin().x();
    map.origin.y = mapper_->origin().y();
    map.origin.z = mapper_->origin().z();
    map.size_x = static_cast<uint32_t>(mapper_->sizeX());
    map.size_y = static_cast<uint32_t>(mapper_->sizeY());
    map.size_z = static_cast<uint32_t>(mapper_->sizeZ());
    map.occupancy = mapper_->occupancy();
    // Valid once we know where the window is (pose received) — an empty
    // but correctly-positioned map is a usable answer ("nothing observed
    // yet"); a map centered on a guessed origin is not.
    map.map_valid = loc_.has_value() && loc_->localization_ok;
    map_pub_->publish(map);

    const auto clusters = uav_world_model::clusterOccupied(
      *mapper_, min_cluster_voxels_, max_cluster_height_m_);
    const auto tracks = tracker_.track(clusters, tick_period_s_);

    ObstacleSet obstacle_set;
    obstacle_set.header.stamp = now;
    obstacle_set.header.frame_id = "map";
    obstacle_set.obstacles.reserve(tracks.size());
    for (const auto & tr : tracks) {
      Obstacle o;
      o.id = tr.id;
      o.position.x = tr.position.x();
      o.position.y = tr.position.y();
      o.position.z = tr.position.z();
      o.velocity.x = tr.velocity.x();
      o.velocity.y = tr.velocity.y();
      o.velocity.z = tr.velocity.z();
      o.radius = static_cast<float>(tr.radius);
      o.obstacle_class = tr.dynamic ? Obstacle::CLASS_DYNAMIC : Obstacle::CLASS_STATIC;
      obstacle_set.obstacles.push_back(o);
    }
    obstacle_pub_->publish(obstacle_set);
  }

  rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<LocalizationState>::SharedPtr loc_sub_;
  rclcpp::Publisher<LocalMap>::SharedPtr map_pub_;
  rclcpp::Publisher<ObstacleSet>::SharedPtr obstacle_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<uav_world_model::VoxelMapper> mapper_;
  uav_world_model::ObstacleTracker tracker_;
  std::optional<LocalizationState> loc_;
  std::vector<Eigen::Vector3d> pending_points_;

  int point_stride_{2};
  int decay_every_n_ticks_{10};
  int min_cluster_voxels_{3};
  double max_cluster_height_m_{1.2};
  int ticks_since_decay_{0};
  double tick_period_s_{0.2};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealWorldModel>());
  rclcpp::shutdown();
  return 0;
}
