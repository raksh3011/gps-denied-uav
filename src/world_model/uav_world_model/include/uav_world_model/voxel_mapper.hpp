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

// VoxelMapper: the rolling local occupancy map behind LocalMap. Like
// uav_planning's Grid3D, deliberately ROS-free: built from plain points
// and poses so it's unit-testable without a ROS runtime; real_world_model
// is the only place that converts PointCloud2 into calls on this class.
//
// Design points that matter downstream (see docs/WORLD_MODEL.md):
// - Hit-count evidence, not single-shot marking: a voxel becomes occupied
//   only after `min_hits` observations, and `decay()` (called periodically
//   by the node) erodes counts so a voxel vacated by a moving obstacle
//   eventually frees itself. LiDAR noise therefore doesn't instantly
//   hallucinate walls, and dynamic obstacles don't leave permanent trails.
// - Chunked re-centering: the map origin follows the vehicle, but only
//   re-centers when the vehicle strays more than `recenter_threshold_m`
//   from the map center, and the shift is a whole number of voxels with
//   surviving evidence block-copied across. Every origin change forces
//   the planner's DStarLitePlanner to a full re-initialize (see
//   docs/PLANNING.md's rolling-map caveat), so re-centering rarely — in
//   multi-meter chunks — instead of continuously is what keeps the local
//   planner's incremental property alive most ticks.
// - Exported occupancy is strictly 0/1 (never 255-unknown): the planner
//   treats any nonzero cell as an obstacle, so publishing unknown-as-255
//   would wall off all unexplored space. Unknown is published as free
//   (optimistic navigation) — a deliberate, documented choice.
#ifndef UAV_WORLD_MODEL__VOXEL_MAPPER_HPP_
#define UAV_WORLD_MODEL__VOXEL_MAPPER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

namespace uav_world_model
{

struct VoxelMapperParams
{
  double resolution{0.2};          // m per voxel, matching LocalMap default
  int size_x{50};                  // voxels; 50 x 0.2 = 10 m window
  int size_y{50};
  int size_z{30};                  // 6 m vertical window
  int min_hits{2};                 // observations before a voxel counts as occupied
  int max_hits{10};                // hit-count saturation (bounds forget time)
  int decay_per_call{1};           // subtracted from every voxel per decay()
  double recenter_threshold_m{2.0};   // recenter when vehicle strays this far from center
};

class VoxelMapper
{
public:
  explicit VoxelMapper(const VoxelMapperParams & params);

  // Shifts the window so `vehicle_pos` is back at its center IF the
  // vehicle has strayed more than recenter_threshold_m from the current
  // center (horizontally or vertically). Shift is a whole number of
  // voxels; evidence still inside the new window is preserved. Returns
  // true if the origin moved (the node uses this purely for logging —
  // the planner detects the origin change itself from the published map).
  bool maybeRecenter(const Eigen::Vector3d & vehicle_pos);

  // Accumulates one hit per point (map-frame). Points outside the current
  // window are ignored.
  void integratePoints(const std::vector<Eigen::Vector3d> & points);

  // Erodes all hit counts by decay_per_call (floor 0). The node calls
  // this at a fixed slow cadence so stale evidence fades.
  void decay();

  // 0 = free (or unknown — see header comment), 1 = occupied.
  // Length size_x*size_y*size_z, row-major with x fastest, matching both
  // the LocalMap contract and uav_planning::Grid3D::loadOccupancy.
  std::vector<uint8_t> occupancy() const;

  bool isOccupied(int x, int y, int z) const;
  bool inBounds(int x, int y, int z) const;
  Eigen::Vector3d voxelCenter(int x, int y, int z) const;

  const Eigen::Vector3d & origin() const {return origin_;}
  double resolution() const {return params_.resolution;}
  int sizeX() const {return params_.size_x;}
  int sizeY() const {return params_.size_y;}
  int sizeZ() const {return params_.size_z;}

private:
  size_t flatten(int x, int y, int z) const;

  VoxelMapperParams params_;
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};   // world coords of voxel (0,0,0) corner
  std::vector<uint8_t> hits_;
};

}  // namespace uav_world_model

#endif  // UAV_WORLD_MODEL__VOXEL_MAPPER_HPP_
