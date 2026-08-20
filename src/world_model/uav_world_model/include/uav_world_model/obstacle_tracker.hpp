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

// Obstacle extraction + tracking behind ObstacleSet. ROS-free, like
// VoxelMapper. Two stages:
// - clusterOccupied(): 26-connected component labeling over the mapper's
//   occupied voxels. A component is then sliced into vertical layers no
//   taller than max_height_m before each layer becomes its own bounding
//   sphere (centroid + max voxel-center distance + half a voxel).
//   Without this slicing, a tall thin obstacle (a pole, a tree trunk, a
//   building corner) gets ONE sphere whose radius is dragged out to cover
//   its full height — e.g. a 0.5m-radius, 2.5m-tall pillar becomes a
//   ~1.5m-radius sphere, more than 2x its true width. After planner
//   margins are added, that false width can close off gaps that are
//   actually flyable. Slicing bounds each sphere's radius to roughly the
//   obstacle's true horizontal footprint. Components/slices smaller than
//   min_voxels are dropped as sensor noise.
// - ObstacleTracker::track(): frame-to-frame association by nearest
//   centroid within a gate, giving stable ids, a smoothed velocity
//   estimate from centroid deltas, and a STATIC/DYNAMIC classification by
//   speed threshold. Deliberately simple nearest-neighbor tracking — no
//   Kalman filter, no assignment optimization; honest v1, limits in
//   docs/WORLD_MODEL.md.
#ifndef UAV_WORLD_MODEL__OBSTACLE_TRACKER_HPP_
#define UAV_WORLD_MODEL__OBSTACLE_TRACKER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

#include "uav_world_model/voxel_mapper.hpp"

namespace uav_world_model
{

struct VoxelCluster
{
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  double radius{0.0};
  int voxel_count{0};
};

// min_voxels: clusters (post-slicing) with fewer occupied voxels are
// discarded as noise. max_height_m: connected components are sliced into
// vertical layers no taller than this before spherizing (see header
// comment above); pass 0 to disable slicing (one sphere per component,
// the old behavior).
std::vector<VoxelCluster> clusterOccupied(
  const VoxelMapper & map, int min_voxels, double max_height_m = 0.0);

struct TrackedObstacle
{
  uint32_t id{0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double radius{0.0};
  bool dynamic{false};
  int missed_frames{0};   // internal: consecutive frames without a match
};

class ObstacleTracker
{
public:
  // gate_m: max centroid distance for frame-to-frame association.
  // dynamic_speed_mps: smoothed speed above which an obstacle is DYNAMIC.
  // max_missed_frames: unmatched tracks are dropped after this many frames.
  ObstacleTracker(double gate_m, double dynamic_speed_mps, int max_missed_frames);

  // dt_s: time since the previous track() call (used for velocity).
  // Returns the current live tracks, ids stable across frames.
  std::vector<TrackedObstacle> track(const std::vector<VoxelCluster> & clusters, double dt_s);

private:
  double gate_m_;
  double dynamic_speed_mps_;
  int max_missed_frames_;
  uint32_t next_id_{1};
  std::vector<TrackedObstacle> tracks_;
};

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__OBSTACLE_TRACKER_HPP_
