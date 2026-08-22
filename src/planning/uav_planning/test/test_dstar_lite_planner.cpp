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
#include <chrono>
#include <limits>
#include <vector>

#include "uav_planning/dstar_lite_planner.hpp"

using uav_planning::DStarLitePlanner;
using uav_planning::Grid3D;
using uav_planning::ObstacleSphere;

TEST(DStarLitePlanner, NotInitializedReturnsEmpty) {
  DStarLitePlanner planner;
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  EXPECT_TRUE(planner.update(grid, Eigen::Vector3d(0.0, 0.0, 1.0)).empty());
}

TEST(DStarLitePlanner, FindsInitialPathOnEmptyGrid) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);

  const auto path = planner.update(grid, start);
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.front() - start).norm(), 0.0, 0.5);
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, ReroutesAroundANewObstacleWithoutReinitializing) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);

  const auto initial_path = planner.update(grid, start);
  ASSERT_FALSE(initial_path.empty());
  // On an empty grid the initial path should hug the direct line.
  for (const auto & point : initial_path) {
    EXPECT_LT(std::abs(point.y()), 0.5);
  }

  // Now an obstacle appears directly on that line — same Grid3D object
  // mutated in place (mirrors how real_planner rebuilds a fresh Grid3D
  // each tick from the latest LocalMap/ObstacleSet), and we call update()
  // again WITHOUT calling initialize() — this is the property under test.
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(1.5, 0.0, 1.0), 0.8}};
  grid.inflateObstacles(obstacles, 0.2, 0.5, 5.0);

  const auto rerouted_path = planner.update(grid, start);
  ASSERT_FALSE(rerouted_path.empty());
  for (const auto & point : rerouted_path) {
    const double dist_to_obstacle = (point - obstacles.front().center).norm();
    EXPECT_GT(dist_to_obstacle, obstacles.front().radius)
      << "rerouted path point (" << point.transpose() << ") lands inside the new obstacle";
  }
}

TEST(DStarLitePlanner, TracksVehicleMovingTowardGoal) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;

  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(4.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);
  planner.update(grid, start);

  // Simulate the vehicle having advanced partway toward the goal.
  const Eigen::Vector3d advanced(2.0, 0.0, 1.0);
  const auto path = planner.update(grid, advanced);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.front() - advanced).norm(), 0.0, 0.5);
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, GoalOccupiedGivesEmptyPath) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 10, 10, 4);
  DStarLitePlanner planner;
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(2.0, 0.0, 1.0);

  // Occupy the goal cell itself before initializing.
  std::vector<ObstacleSphere> obstacles = {{goal, 0.3}};
  grid.inflateObstacles(obstacles, 0.0, 0.0, 0.0);

  planner.initialize(grid, start, goal);
  EXPECT_TRUE(planner.update(grid, start).empty());
}

TEST(DStarLitePlanner, AlreadyPastDeadlineStopsImmediatelyAndSafely) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);   // unbounded, off the real-time path

  // A deadline already in the past: computeShortestPath() must bail out
  // on its very first check, report it via lastComputeHitDeadline(), and
  // never corrupt state (a subsequent unbounded call still converges).
  const auto past = DStarLitePlanner::Clock::now() - std::chrono::seconds(1);
  const auto path = planner.update(grid, start, past);
  EXPECT_TRUE(planner.lastComputeHitDeadline());
  // Safe either way: empty (didn't converge in time) or already-correct
  // from initialize()'s own unbounded pass — never a corrupt/garbage one.
  if (!path.empty()) {
    EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
  }

  const auto recovered = planner.update(grid, start, DStarLitePlanner::kNoDeadline);
  EXPECT_FALSE(planner.lastComputeHitDeadline());
  ASSERT_FALSE(recovered.empty());
  EXPECT_NEAR((recovered.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, GenerousDeadlineConvergesNormallyAndReportsSo) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner;
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);

  const auto generous = DStarLitePlanner::Clock::now() + std::chrono::seconds(5);
  const auto path = planner.update(grid, start, generous);
  EXPECT_FALSE(planner.lastComputeHitDeadline());
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, InterruptedSearchResumesAndConvergesAcrossCalls) {
  // Deliberately deterministic instead of timing-based: a tiny expansion
  // cap plays the same role a tight deadline would (both leave
  // g_/rhs_/queue_ mid-search, which is the property under test — that
  // this is safe to interrupt and resume, whichever cap triggers it) but
  // without any wall-clock flakiness in CI.
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  DStarLitePlanner planner(/*max_compute_iterations=*/2);
  const Eigen::Vector3d start(0.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);
  planner.initialize(grid, start, goal);

  // Keep calling update() (each call resumes from where the last left
  // off, per D* Lite's own incremental invariant) until it converges —
  // must happen well within a generous number of ticks, proving the
  // search makes real forward progress each time rather than restarting
  // or stalling.
  std::vector<Eigen::Vector3d> path;
  for (int tick = 0; tick < 200 && path.empty(); ++tick) {
    path = planner.update(grid, start);
  }
  ASSERT_FALSE(path.empty()) << "capped search never converged across repeated resumed calls";
  EXPECT_NEAR((path.back() - goal).norm(), 0.0, 0.5);
}

TEST(DStarLitePlanner, RiskBandQuantizationMapsConfidenceAndStatus) {
  DStarLitePlanner planner;
  // NOMINAL status, high confidence -> baseline multiplier.
  planner.setLocalizationRisk(0.95F, 0);
  EXPECT_DOUBLE_EQ(planner.riskMultiplier(), 1.0);
  // Small confidence wobble inside the same band -> no change (quantized).
  planner.setLocalizationRisk(0.85F, 0);
  EXPECT_DOUBLE_EQ(planner.riskMultiplier(), 1.0);
  // Marginal confidence -> middle band.
  planner.setLocalizationRisk(0.5F, 0);
  EXPECT_DOUBLE_EQ(planner.riskMultiplier(), 2.0);
  // Very low confidence -> highest band.
  planner.setLocalizationRisk(0.2F, 0);
  EXPECT_DOUBLE_EQ(planner.riskMultiplier(), 4.0);
  // Status escalates the band even when confidence looks fine.
  planner.setLocalizationRisk(0.95F, 1);   // STATUS_DEGRADED
  EXPECT_DOUBLE_EQ(planner.riskMultiplier(), 2.0);
  planner.setLocalizationRisk(0.95F, 2);   // STATUS_LOST
  EXPECT_DOUBLE_EQ(planner.riskMultiplier(), 4.0);
}

TEST(DStarLitePlanner, DegradedLocalizationPrefersWiderBerth) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  // Obstacle astride the direct line, with a wide soft-cost apron.
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(0.5, 0.0, 1.0), 0.6}};
  grid.inflateObstacles(obstacles, 0.2, 1.5, 2.0);

  const Eigen::Vector3d start(-2.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);

  auto soft_cost_along = [&grid](const std::vector<Eigen::Vector3d> & path) {
      double sum = 0.0;
      for (const auto & p : path) {
        sum += grid.traversalCost(grid.worldToIndex(p));
      }
      return sum;
    };

  DStarLitePlanner nominal;
  nominal.initialize(grid, start, goal);
  nominal.setLocalizationRisk(0.95F, 0);
  const auto nominal_path = nominal.update(grid, start);
  ASSERT_FALSE(nominal_path.empty());

  DStarLitePlanner lost;
  lost.initialize(grid, start, goal);
  lost.setLocalizationRisk(0.1F, 2);   // STATUS_LOST
  const auto lost_path = lost.update(grid, start);
  ASSERT_FALSE(lost_path.empty());

  // Under degraded localization the planner must not cut closer to the
  // obstacle than it did when localization was healthy — and it should
  // accumulate no more raw proximity cost than the nominal path did.
  auto min_clearance = [&obstacles](const std::vector<Eigen::Vector3d> & path) {
      double best = std::numeric_limits<double>::infinity();
      for (const auto & p : path) {
        best = std::min(best, (p - obstacles.front().center).norm());
      }
      return best;
    };
  EXPECT_GE(min_clearance(lost_path) + 1e-9, min_clearance(nominal_path));
  EXPECT_LE(soft_cost_along(lost_path), soft_cost_along(nominal_path) + 1e-9);
}

TEST(DStarLitePlanner, RiskBandChangeMidFlightReroutesIncrementally) {
  Grid3D grid(0.5, Eigen::Vector3d(-5.0, -5.0, 0.0), 20, 20, 4);
  std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(0.5, 0.0, 1.0), 0.6}};
  grid.inflateObstacles(obstacles, 0.2, 1.5, 2.0);

  const Eigen::Vector3d start(-2.0, 0.0, 1.0);
  const Eigen::Vector3d goal(3.0, 0.0, 1.0);

  DStarLitePlanner planner;
  planner.initialize(grid, start, goal);
  const auto before = planner.update(grid, start);
  ASSERT_FALSE(before.empty());

  // Localization degrades mid-flight — no re-initialize, same planner.
  planner.setLocalizationRisk(0.1F, 2);
  const auto after = planner.update(grid, start);
  ASSERT_FALSE(after.empty());
  EXPECT_TRUE(planner.isInitialized());
  EXPECT_NEAR((after.front() - start).norm(), 0.0, 0.5);
  EXPECT_NEAR((after.back() - goal).norm(), 0.0, 0.5);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
