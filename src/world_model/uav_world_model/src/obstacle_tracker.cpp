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

#include "uav_world_model/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <vector>

namespace uav_world_model
{

namespace
{
struct Voxel
{
  int x;
  int y;
  int z;
};

// Builds one VoxelCluster from a set of voxels already known to belong
// together (either a whole connected component, or one vertical slice of
// one). Returns std::nullopt if fewer than min_voxels members.
std::optional<VoxelCluster> spherize(
  const VoxelMapper & map, const std::vector<Voxel> & members, int min_voxels)
{
  if (static_cast<int>(members.size()) < min_voxels) {return std::nullopt;}

  VoxelCluster cluster;
  cluster.voxel_count = static_cast<int>(members.size());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & m : members) {
    sum += map.voxelCenter(m.x, m.y, m.z);
  }
  cluster.centroid = sum / static_cast<double>(members.size());
  double max_dist = 0.0;
  for (const auto & m : members) {
    max_dist = std::max(max_dist, (map.voxelCenter(m.x, m.y, m.z) - cluster.centroid).norm());
  }
  // Half a voxel on top so the sphere covers the outermost voxel's own
  // extent, not just its center.
  cluster.radius = max_dist + 0.5 * map.resolution();
  return cluster;
}
}  // namespace

std::vector<VoxelCluster> clusterOccupied(
  const VoxelMapper & map, int min_voxels, double max_height_m)
{
  const int sx = map.sizeX();
  const int sy = map.sizeY();
  const int sz = map.sizeZ();
  std::vector<bool> visited(static_cast<size_t>(sx) * sy * sz, false);
  auto flat = [sx, sy](int x, int y, int z) {
      return static_cast<size_t>(x) + static_cast<size_t>(y) * sx +
             static_cast<size_t>(z) * sx * sy;
    };
  // 0 or negative disables slicing: treat the whole component as one bin.
  const int slice_voxels = max_height_m > 0.0 ?
    std::max(1, static_cast<int>(std::round(max_height_m / map.resolution()))) :
    std::numeric_limits<int>::max();

  std::vector<VoxelCluster> clusters;

  for (int z0 = 0; z0 < sz; ++z0) {
    for (int y0 = 0; y0 < sy; ++y0) {
      for (int x0 = 0; x0 < sx; ++x0) {
        if (visited[flat(x0, y0, z0)] || !map.isOccupied(x0, y0, z0)) {continue;}

        // BFS flood-fill over the 26-neighborhood.
        std::vector<Voxel> members;
        std::queue<Voxel> frontier;
        frontier.push({x0, y0, z0});
        visited[flat(x0, y0, z0)] = true;
        while (!frontier.empty()) {
          const Voxel v = frontier.front();
          frontier.pop();
          members.push_back(v);
          for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {continue;}
                const int nx = v.x + dx;
                const int ny = v.y + dy;
                const int nz = v.z + dz;
                if (!map.inBounds(nx, ny, nz) || visited[flat(nx, ny, nz)]) {continue;}
                if (!map.isOccupied(nx, ny, nz)) {continue;}
                visited[flat(nx, ny, nz)] = true;
                frontier.push({nx, ny, nz});
              }
            }
          }
        }

        if (members.empty()) {continue;}

        // Slice into vertical layers by z-bin, so a tall component
        // becomes several stacked spheres instead of one over-wide one
        // (see the "why" in the header comment above clusterOccupied).
        const int base_z = members.front().z;
        std::map<int, std::vector<Voxel>> bins;
        for (const auto & m : members) {
          bins[(m.z - base_z) / slice_voxels].push_back(m);
        }
        for (const auto & [bin_index, bin_members] : bins) {
          (void)bin_index;
          if (auto cluster = spherize(map, bin_members, min_voxels)) {
            clusters.push_back(*cluster);
          }
        }
      }
    }
  }
  return clusters;
}

ObstacleTracker::ObstacleTracker(double gate_m, double dynamic_speed_mps, int max_missed_frames)
: gate_m_(gate_m), dynamic_speed_mps_(dynamic_speed_mps), max_missed_frames_(max_missed_frames)
{
}

std::vector<TrackedObstacle> ObstacleTracker::track(
  const std::vector<VoxelCluster> & clusters, double dt_s)
{
  std::vector<bool> cluster_claimed(clusters.size(), false);

  // Greedy nearest-neighbor: each existing track claims its closest
  // unclaimed cluster within the gate, in track order.
  for (auto & tr : tracks_) {
    int best = -1;
    double best_dist = gate_m_;
    for (size_t c = 0; c < clusters.size(); ++c) {
      if (cluster_claimed[c]) {continue;}
      const double d = (clusters[c].centroid - tr.position).norm();
      if (d < best_dist) {
        best_dist = d;
        best = static_cast<int>(c);
      }
    }
    if (best < 0) {
      ++tr.missed_frames;
      continue;
    }
    cluster_claimed[best] = true;
    const auto & cl = clusters[static_cast<size_t>(best)];
    if (dt_s > 1e-6) {
      const Eigen::Vector3d instantaneous = (cl.centroid - tr.position) / dt_s;
      // Exponential smoothing so one noisy centroid doesn't flip the class.
      tr.velocity = 0.7 * tr.velocity + 0.3 * instantaneous;
    }
    tr.position = cl.centroid;
    tr.radius = cl.radius;
    tr.dynamic = tr.velocity.norm() > dynamic_speed_mps_;
    tr.missed_frames = 0;
  }

  // Unclaimed clusters start new tracks (born static, zero velocity).
  for (size_t c = 0; c < clusters.size(); ++c) {
    if (cluster_claimed[c]) {continue;}
    TrackedObstacle tr;
    tr.id = next_id_++;
    tr.position = clusters[c].centroid;
    tr.radius = clusters[c].radius;
    tracks_.push_back(tr);
  }

  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(),
      [this](const TrackedObstacle & tr) {return tr.missed_frames > max_missed_frames_;}),
    tracks_.end());

  std::vector<TrackedObstacle> live;
  live.reserve(tracks_.size());
  for (const auto & tr : tracks_) {
    if (tr.missed_frames == 0) {live.push_back(tr);}
  }
  return live;
}

}  // namespace uav_world_model
