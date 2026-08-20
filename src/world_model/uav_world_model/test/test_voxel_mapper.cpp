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

#include <cmath>
#include <vector>

#include "uav_world_model/voxel_mapper.hpp"

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
  p.min_hits = 2;
  p.max_hits = 10;
  p.decay_per_call = 1;
  p.recenter_threshold_m = 2.0;
  return p;
}
}  // namespace

TEST(VoxelMapper, RequiresMinHitsBeforeOccupied) {
  VoxelMapper mapper(smallParams());
  const std::vector<Eigen::Vector3d> hit = {{1.0, 1.0, 1.0}};

  mapper.integratePoints(hit);   // 1 hit < min_hits(2)
  auto occ = mapper.occupancy();
  for (uint8_t v : occ) {
    EXPECT_EQ(v, 0);
  }

  mapper.integratePoints(hit);   // 2nd hit crosses the threshold
  occ = mapper.occupancy();
  int occupied = 0;
  for (uint8_t v : occ) {
    occupied += v;
  }
  EXPECT_EQ(occupied, 1);
}

TEST(VoxelMapper, OccupancyLayoutMatchesGridConvention) {
  // The exported flat array must be x-fastest row-major, exactly what the
  // LocalMap contract states and uav_planning::Grid3D::loadOccupancy expects.
  VoxelMapper mapper(smallParams());
  const Eigen::Vector3d p(1.3, 2.1, 0.7);   // some arbitrary in-window point
  mapper.integratePoints({p, p});

  const Eigen::Vector3d rel = p - mapper.origin();
  const int x = static_cast<int>(rel.x() / 0.5);
  const int y = static_cast<int>(rel.y() / 0.5);
  const int z = static_cast<int>(rel.z() / 0.5);
  ASSERT_TRUE(mapper.isOccupied(x, y, z));

  const auto occ = mapper.occupancy();
  const size_t flat = static_cast<size_t>(x) + static_cast<size_t>(y) * 20 +
    static_cast<size_t>(z) * 20 * 20;
  EXPECT_EQ(occ[flat], 1);
}

TEST(VoxelMapper, DecayFreesStaleVoxels) {
  VoxelMapper mapper(smallParams());
  const std::vector<Eigen::Vector3d> hit = {{1.0, 1.0, 1.0}};
  mapper.integratePoints(hit);
  mapper.integratePoints(hit);
  const Eigen::Vector3d rel = Eigen::Vector3d(1.0, 1.0, 1.0) - mapper.origin();
  const int x = static_cast<int>(rel.x() / 0.5);
  const int y = static_cast<int>(rel.y() / 0.5);
  const int z = static_cast<int>(rel.z() / 0.5);
  ASSERT_TRUE(mapper.isOccupied(x, y, z));

  mapper.decay();   // 2 -> 1, below min_hits
  EXPECT_FALSE(mapper.isOccupied(x, y, z));
}

TEST(VoxelMapper, IgnoresPointsOutsideWindow) {
  VoxelMapper mapper(smallParams());
  const std::vector<Eigen::Vector3d> far = {{100.0, 100.0, 100.0}, {100.0, 100.0, 100.0}};
  mapper.integratePoints(far);
  for (uint8_t v : mapper.occupancy()) {
    EXPECT_EQ(v, 0);
  }
}

TEST(VoxelMapper, NoRecenterInsideThreshold) {
  VoxelMapper mapper(smallParams());
  const Eigen::Vector3d origin_before = mapper.origin();
  EXPECT_FALSE(mapper.maybeRecenter(Eigen::Vector3d(1.0, 1.0, 0.5)));
  EXPECT_EQ(mapper.origin(), origin_before);
}

TEST(VoxelMapper, RecenterShiftsWindowAndPreservesEvidence) {
  VoxelMapper mapper(smallParams());
  // Occupy a voxel near where the vehicle is heading, so it survives.
  const Eigen::Vector3d obstacle(3.0, 0.0, 0.5);
  mapper.integratePoints({obstacle, obstacle});

  // Vehicle moves +3m in x — beyond the 2m threshold.
  ASSERT_TRUE(mapper.maybeRecenter(Eigen::Vector3d(3.0, 0.0, 0.0)));

  // Origin moved by a whole number of voxels.
  const Eigen::Vector3d origin = mapper.origin();
  const double shift_voxels = (origin.x() + 5.0) / 0.5;   // initial origin.x was -5.0
  EXPECT_NEAR(shift_voxels, std::round(shift_voxels), 1e-9);

  // The obstacle's evidence survived the shift at its (new) index.
  const Eigen::Vector3d rel = obstacle - origin;
  const int x = static_cast<int>(rel.x() / 0.5);
  const int y = static_cast<int>(rel.y() / 0.5);
  const int z = static_cast<int>(rel.z() / 0.5);
  EXPECT_TRUE(mapper.isOccupied(x, y, z));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
