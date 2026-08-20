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

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "uav_world_model/obstacle_tracker.hpp"

using uav_world_model::clusterOccupied;
using uav_world_model::ObstacleTracker;
using uav_world_model::VoxelCluster;
using uav_world_model::VoxelMapper;
using uav_world_model::VoxelMapperParams;

namespace
{
VoxelMapperParams smallParams()
{
  VoxelMapperParams p;
  p.resolution = 0.5;
  p.size_x = 20;
  p.size_y = 20;
  p.size_z = 8;
  p.min_hits = 1;   // single hit occupies, simplifies cluster setup
  return p;
}

// Stamps a solid axis-aligned blob of occupied voxels around `center`.
void stampBlob(VoxelMapper & mapper, const Eigen::Vector3d & center, double half_extent)
{
  std::vector<Eigen::Vector3d> points;
  for (double dx = -half_extent; dx <= half_extent; dx += 0.25) {
    for (double dy = -half_extent; dy <= half_extent; dy += 0.25) {
      for (double dz = -half_extent; dz <= half_extent; dz += 0.25) {
        points.push_back(center + Eigen::Vector3d(dx, dy, dz));
      }
    }
  }
  mapper.integratePoints(points);
}
}  // namespace

TEST(ClusterOccupied, TwoSeparatedBlobsGiveTwoClusters) {
  VoxelMapper mapper(smallParams());
  const Eigen::Vector3d a(-2.0, -2.0, 1.0);
  const Eigen::Vector3d b(2.0, 2.0, 1.0);
  stampBlob(mapper, a, 0.5);
  stampBlob(mapper, b, 0.5);

  const auto clusters = clusterOccupied(mapper, 2);
  ASSERT_EQ(clusters.size(), 2u);
  // Each centroid should be near one of the blob centers.
  for (const auto & cluster : clusters) {
    const double da = (cluster.centroid - a).norm();
    const double db = (cluster.centroid - b).norm();
    EXPECT_LT(std::min(da, db), 0.6);
    EXPECT_GT(cluster.radius, 0.0);
  }
}

TEST(ClusterOccupied, MinVoxelsFiltersNoise) {
  VoxelMapper mapper(smallParams());
  // Single isolated occupied voxel = 1-voxel cluster = noise.
  mapper.integratePoints({{1.0, 1.0, 1.0}});
  EXPECT_TRUE(clusterOccupied(mapper, 2).empty());
  EXPECT_EQ(clusterOccupied(mapper, 1).size(), 1u);
}

TEST(ObstacleTracker, StableIdAcrossFramesAndStaticClass) {
  ObstacleTracker tracker(1.0, 0.3, 3);
  VoxelCluster cluster;
  cluster.centroid = Eigen::Vector3d(2.0, 0.0, 1.0);
  cluster.radius = 0.5;
  cluster.voxel_count = 8;

  const auto frame1 = tracker.track({cluster}, 0.2);
  ASSERT_EQ(frame1.size(), 1u);
  const auto id = frame1.front().id;

  const auto frame2 = tracker.track({cluster}, 0.2);
  ASSERT_EQ(frame2.size(), 1u);
  EXPECT_EQ(frame2.front().id, id);
  EXPECT_FALSE(frame2.front().dynamic);
}

TEST(ObstacleTracker, MovingClusterBecomesDynamicWithVelocity) {
  ObstacleTracker tracker(2.0, 0.3, 3);
  VoxelCluster cluster;
  cluster.centroid = Eigen::Vector3d(0.0, 0.0, 1.0);
  cluster.radius = 0.5;
  cluster.voxel_count = 8;

  tracker.track({cluster}, 0.2);
  // Move 0.4m per 0.2s frame = 2 m/s, well above the 0.3 m/s threshold.
  for (int i = 1; i <= 5; ++i) {
    cluster.centroid = Eigen::Vector3d(0.4 * i, 0.0, 1.0);
    const auto tracks = tracker.track({cluster}, 0.2);
    ASSERT_EQ(tracks.size(), 1u);
    if (i >= 3) {   // smoothing needs a few frames to converge
      EXPECT_TRUE(tracks.back().dynamic);
      EXPECT_GT(tracks.back().velocity.x(), 0.3);
    }
  }
}

TEST(ObstacleTracker, VanishedObstacleIsDroppedAfterMissedFrames) {
  ObstacleTracker tracker(1.0, 0.3, 2);
  VoxelCluster cluster;
  cluster.centroid = Eigen::Vector3d(2.0, 0.0, 1.0);
  cluster.radius = 0.5;
  cluster.voxel_count = 8;

  tracker.track({cluster}, 0.2);
  // Obstacle disappears; for max_missed_frames(2)+1 frames nothing matches.
  EXPECT_TRUE(tracker.track({}, 0.2).empty());
  EXPECT_TRUE(tracker.track({}, 0.2).empty());
  EXPECT_TRUE(tracker.track({}, 0.2).empty());

  // A new appearance in the same place gets a NEW id (old track dropped).
  const auto tracks = tracker.track({cluster}, 0.2);
  ASSERT_EQ(tracks.size(), 1u);
  EXPECT_NE(tracks.front().id, 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
