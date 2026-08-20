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

#include "uav_world_model/voxel_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace uav_world_model
{

VoxelMapper::VoxelMapper(const VoxelMapperParams & params)
: params_(params)
{
  const size_t n = static_cast<size_t>(params_.size_x) * params_.size_y * params_.size_z;
  hits_.assign(n, 0);
  // Initial window centered on the world origin until the first recenter.
  origin_ = Eigen::Vector3d(
    -0.5 * params_.size_x * params_.resolution,
    -0.5 * params_.size_y * params_.resolution,
    -0.5 * params_.size_z * params_.resolution);
}

size_t VoxelMapper::flatten(int x, int y, int z) const
{
  return static_cast<size_t>(x) + static_cast<size_t>(y) * params_.size_x +
    static_cast<size_t>(z) * params_.size_x * params_.size_y;
}

bool VoxelMapper::inBounds(int x, int y, int z) const
{
  return x >= 0 && x < params_.size_x && y >= 0 && y < params_.size_y &&
    z >= 0 && z < params_.size_z;
}

Eigen::Vector3d VoxelMapper::voxelCenter(int x, int y, int z) const
{
  return origin_ + Eigen::Vector3d(
    (x + 0.5) * params_.resolution,
    (y + 0.5) * params_.resolution,
    (z + 0.5) * params_.resolution);
}

bool VoxelMapper::maybeRecenter(const Eigen::Vector3d & vehicle_pos)
{
  const Eigen::Vector3d center = origin_ + Eigen::Vector3d(
    0.5 * params_.size_x * params_.resolution,
    0.5 * params_.size_y * params_.resolution,
    0.5 * params_.size_z * params_.resolution);
  const Eigen::Vector3d offset = vehicle_pos - center;
  if (std::abs(offset.x()) < params_.recenter_threshold_m &&
    std::abs(offset.y()) < params_.recenter_threshold_m &&
    std::abs(offset.z()) < params_.recenter_threshold_m)
  {
    return false;
  }

  // Whole-voxel shift that puts the vehicle back at (as close as possible
  // to) the window center. Positive shift = window moves in +axis, i.e.
  // evidence moves to lower indices.
  const int shift_x = static_cast<int>(std::llround(offset.x() / params_.resolution));
  const int shift_y = static_cast<int>(std::llround(offset.y() / params_.resolution));
  const int shift_z = static_cast<int>(std::llround(offset.z() / params_.resolution));
  if (shift_x == 0 && shift_y == 0 && shift_z == 0) {
    return false;
  }

  std::vector<uint8_t> shifted(hits_.size(), 0);
  for (int z = 0; z < params_.size_z; ++z) {
    const int src_z = z + shift_z;
    if (src_z < 0 || src_z >= params_.size_z) {continue;}
    for (int y = 0; y < params_.size_y; ++y) {
      const int src_y = y + shift_y;
      if (src_y < 0 || src_y >= params_.size_y) {continue;}
      for (int x = 0; x < params_.size_x; ++x) {
        const int src_x = x + shift_x;
        if (src_x < 0 || src_x >= params_.size_x) {continue;}
        shifted[flatten(x, y, z)] = hits_[flatten(src_x, src_y, src_z)];
      }
    }
  }
  hits_ = std::move(shifted);
  origin_ += Eigen::Vector3d(
    shift_x * params_.resolution,
    shift_y * params_.resolution,
    shift_z * params_.resolution);
  return true;
}

void VoxelMapper::integratePoints(const std::vector<Eigen::Vector3d> & points)
{
  for (const auto & p : points) {
    const Eigen::Vector3d rel = p - origin_;
    const int x = static_cast<int>(std::floor(rel.x() / params_.resolution));
    const int y = static_cast<int>(std::floor(rel.y() / params_.resolution));
    const int z = static_cast<int>(std::floor(rel.z() / params_.resolution));
    if (!inBounds(x, y, z)) {continue;}
    uint8_t & h = hits_[flatten(x, y, z)];
    if (h < params_.max_hits) {++h;}
  }
}

void VoxelMapper::decay()
{
  for (auto & h : hits_) {
    h = static_cast<uint8_t>(std::max(0, static_cast<int>(h) - params_.decay_per_call));
  }
}

bool VoxelMapper::isOccupied(int x, int y, int z) const
{
  if (!inBounds(x, y, z)) {return false;}
  return hits_[flatten(x, y, z)] >= params_.min_hits;
}

std::vector<uint8_t> VoxelMapper::occupancy() const
{
  std::vector<uint8_t> out(hits_.size(), 0);
  for (size_t i = 0; i < hits_.size(); ++i) {
    out[i] = hits_[i] >= params_.min_hits ? 1 : 0;
  }
  return out;
}

}  // namespace uav_world_model
