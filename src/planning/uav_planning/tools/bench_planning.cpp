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

// bench_planning: produces MEASURED numbers for docs/PLANNING.md's
// "Benchmarked insights" section — not asserted, not estimated. Not a
// test (no pass/fail, not run by colcon test/CI) — a standalone tool you
// run and read the printed numbers from. ROS-free, same as the library
// it exercises; run directly or via `ros2 run uav_planning bench_planning`.
//
// What it measures, and why each one is the honest comparison to make:
//  1. Cold from-scratch search cost (AStarPlanner vs ThetaStarPlanner) at
//     the actual visual-demo arena scale — the exact scenario that
//     exposed the eager-Theta*/stale-queue bugs (see docs/PLANNING.md).
//  2. DStarLitePlanner steady-state incremental update() cost vs. what a
//     naive "replan from scratch every tick" design would pay instead
//     (repeatedly re-running ThetaStarPlanner) — this is the actual,
//     measured payoff of being incremental, not an asserted one.
//  3. CARM's risk-band-switch cost (setLocalizationRisk re-keying only
//     risk_cells_) vs. a full re-initialize — the actual, measured
//     payoff of CARM applying a scalar multiplier instead of recomputing
//     inflation geometry.
//  4. The deadline hardening's real worst-case latency under an
//     artificially tiny deadline, confirming empirically (not just by
//     construction argument) that a tick never runs away.
//  5. Approximate per-tick memory footprint of DStarLitePlanner's own
//     state at this grid scale.
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "uav_planning/astar_planner.hpp"
#include "uav_planning/dstar_lite_planner.hpp"
#include "uav_planning/grid3d.hpp"
#include "uav_planning/theta_star_planner.hpp"

using Clock = std::chrono::steady_clock;
using uav_planning::AStarPlanner;
using uav_planning::DStarLitePlanner;
using uav_planning::Grid3D;
using uav_planning::ObstacleSphere;
using uav_planning::ThetaStarPlanner;

namespace
{

double ms(Clock::duration d)
{
  return std::chrono::duration<double, std::milli>(d).count();
}

// Same arena as ThetaStarPlanner's FindsPathOnLargeOpenGridWithSparseObstacles
// regression test and demo_mission.launch.xml — the real scale that
// surfaced both fixed bugs, not a toy grid.
Grid3D makeArenaGrid()
{
  Grid3D grid(0.25, Eigen::Vector3d(-15.0, -7.5, -5.0), 120, 60, 40);
  const std::vector<ObstacleSphere> obstacles = {
    {Eigen::Vector3d(-6.0, 1.5, 1.25), 0.5},
    {Eigen::Vector3d(-6.0, -3.5, 1.25), 0.5},
    {Eigen::Vector3d(-2.0, -1.0, 1.25), 0.6},
    {Eigen::Vector3d(-2.0, 4.0, 1.25), 0.4},
    {Eigen::Vector3d(2.0, 1.0, 1.25), 0.5},
    {Eigen::Vector3d(2.0, -4.0, 1.25), 0.4},
    {Eigen::Vector3d(6.0, -1.5, 1.25), 0.5},
    {Eigen::Vector3d(6.0, 3.0, 1.25), 0.5},
  };
  grid.inflateObstacles(obstacles, 0.3, 1.5, 5.0);
  return grid;
}

const Eigen::Vector3d kStart(-10.0, 0.0, 1.5);
const Eigen::Vector3d kGoal(10.0, 0.0, 3.0);

template<typename Fn>
double timeMs(Fn && fn)
{
  const auto t0 = Clock::now();
  fn();
  return ms(Clock::now() - t0);
}

}  // namespace

int main()
{
  std::printf("=== uav_planning bench: arena-scale (120x60x40 @ 0.25m, 8 obstacles) ===\n\n");

  // --- 1. Cold from-scratch search ---
  {
    Grid3D grid = makeArenaGrid();
    AStarPlanner astar;
    ThetaStarPlanner theta;
    double astar_ms = 0.0;
    double theta_ms = 0.0;
    std::vector<Eigen::Vector3d> astar_path;
    std::vector<Eigen::Vector3d> theta_path;
    astar_ms = timeMs([&]() {astar_path = astar.plan(grid, kStart, kGoal);});
    theta_ms = timeMs([&]() {theta_path = theta.plan(grid, kStart, kGoal);});

    double astar_len = 0.0;
    for (size_t i = 1; i < astar_path.size(); ++i) {
      astar_len += (astar_path[i] - astar_path[i - 1]).norm();
    }
    double theta_len = 0.0;
    for (size_t i = 1; i < theta_path.size(); ++i) {
      theta_len += (theta_path[i] - theta_path[i - 1]).norm();
    }

    std::printf("[1] Cold global search (once per goal change)\n");
    std::printf(
      "    AStarPlanner:   %8.3f ms, path found=%s, length=%.2f m\n",
      astar_ms, astar_path.empty() ? "no" : "yes", astar_len);
    std::printf(
      "    ThetaStarPlanner: %6.3f ms, path found=%s, length=%.2f m (Lazy Theta*; the eager\n"
      "                      version took ~150,000 ms and still failed on this exact arena)\n\n",
      theta_ms, theta_path.empty() ? "no" : "yes", theta_len);
  }

  // --- 2. Incremental steady-state vs. naive full-replan-every-tick ---
  {
    Grid3D grid = makeArenaGrid();
    DStarLitePlanner dstar;
    const double init_ms = timeMs([&]() {dstar.initialize(grid, kStart, kGoal);});

    constexpr int kTicks = 50;
    double dstar_total_ms = 0.0;
    double naive_total_ms = 0.0;
    Eigen::Vector3d pos = kStart;
    const Eigen::Vector3d step = (kGoal - kStart).normalized() * 0.05;   // slow crawl, worst case
    for (int i = 0; i < kTicks; ++i) {
      pos += step;
      dstar_total_ms += timeMs([&]() {dstar.update(grid, pos);});

      ThetaStarPlanner naive;
      naive_total_ms += timeMs([&]() {naive.plan(grid, pos, kGoal);});
    }

    std::printf(
      "[2] Incremental steady-state vs. naive full-replan-every-tick (%d ticks)\n", kTicks);
    std::printf("    D* Lite initialize() (once):        %8.3f ms\n", init_ms);
    std::printf(
      "    D* Lite update() avg/tick:           %8.3f ms\n", dstar_total_ms / kTicks);
    std::printf(
      "    Naive Theta*-every-tick avg/tick:    %8.3f ms  (%.1fx slower per tick)\n\n",
      naive_total_ms / kTicks, naive_total_ms / std::max(dstar_total_ms, 1e-6));
  }

  // --- 3. CARM risk-band switch vs. full re-initialize ---
  {
    Grid3D grid = makeArenaGrid();
    DStarLitePlanner dstar;
    dstar.initialize(grid, kStart, kGoal);

    const double band_switch_ms = timeMs(
      [&]() {dstar.setLocalizationRisk(0.5F, 1 /* STATUS_DEGRADED */);});
    const double reinit_ms = timeMs([&]() {dstar.initialize(grid, kStart, kGoal);});

    std::printf("[3] CARM risk-band switch vs. a full re-initialize\n");
    std::printf("    setLocalizationRisk() band change: %8.3f ms\n", band_switch_ms);
    std::printf(
      "    Full initialize() (what a naive design would need instead): %8.3f ms (%.1fx more)\n\n",
      reinit_ms, reinit_ms / std::max(band_switch_ms, 1e-6));
  }

  // --- 4. Deadline hardening's actual worst-case latency ---
  {
    Grid3D grid = makeArenaGrid();
    DStarLitePlanner dstar;
    dstar.initialize(grid, kStart, kGoal);

    constexpr auto kTinyBudget = std::chrono::microseconds(200);
    double worst_ms = 0.0;
    Eigen::Vector3d pos = kStart;
    const Eigen::Vector3d step = (kGoal - kStart).normalized() * 0.05;
    for (int i = 0; i < 50; ++i) {
      pos += step;
      const auto deadline = Clock::now() + kTinyBudget;
      const double call_ms = timeMs([&]() {dstar.update(grid, pos, deadline);});
      worst_ms = std::max(worst_ms, call_ms);
    }
    std::printf(
      "[4] Deadline hardening under an artificially tiny budget (%" PRId64 " us requested)\n",
      static_cast<int64_t>(kTinyBudget.count()));
    std::printf(
      "    Worst observed single-call latency: %8.3f ms (must stay near the requested budget,\n"
      "                                          not blow up with grid size)\n\n", worst_ms);
  }

  // --- 5. Approximate memory footprint ---
  {
    const size_t n = static_cast<size_t>(120) * 60 * 40;
    const size_t bytes =
      n * sizeof(bool) +      // occupied_snapshot_
      n * sizeof(double) +    // cost_snapshot_
      n * sizeof(double) +    // g_
      n * sizeof(double) +    // rhs_
      n * (sizeof(double) * 2) +   // best_key_ (Key{k1,k2})
      n * sizeof(bool);       // in_queue_
    std::printf("[5] Approximate DStarLitePlanner state footprint at this grid scale\n");
    std::printf(
      "    %zu cells -> ~%.2f MB (excludes the priority queue and risk_cells_,\n"
      "    both bounded by cells-with-soft-cost, not total grid size)\n",
      n, bytes / (1024.0 * 1024.0));
  }

  return 0;
}
