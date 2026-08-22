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

// DStarLitePlanner: incremental replanning (Koenig & Likhachev, "D* Lite",
// 2002) — the local planner. Unlike AStarPlanner/ThetaStarPlanner, which
// search from scratch every call, this holds search state (g/rhs values,
// priority queue) across calls and only re-examines the cells whose
// occupancy/cost actually changed since the last update, plus whatever
// propagates from them. That's the property that matters for a local
// planner: bounded, low-latency reaction to a moving obstacle or a
// shifted vehicle position, instead of paying for a full re-search on
// every tick regardless of how much of the map actually changed.
//
// Usage: call initialize() once (on mission start, or whenever the goal
// changes — D* Lite's incremental machinery is specifically NOT valid
// across a goal change, only across start motion / graph changes), then
// update() every tick with the current position and latest grid.
//
// Real-time hardening: update()/setLocalizationRisk() take an optional
// wall-clock deadline (steady_clock::time_point, default = none). This is
// NOT presented as a novel algorithm — Anytime Dynamic A* (Likhachev,
// Ferguson, Gordon, Stentz, Thrun, 2005) already combines bounded-time
// anytime search with incremental replanning, and a naive "add a
// deadline" change would just be re-deriving that territory. What's here
// is narrower and honestly scoped: D* Lite's own invariant is that its
// g/rhs state and priority queue are valid to interrupt and resume at
// any point (that's the whole point of an incremental planner), so
// bounding computeShortestPath() by wall-clock time — in addition to the
// existing max_compute_iterations expansion cap — guarantees a tick
// never blocks the control loop past its period, at the cost of
// possibly reporting a stale/suboptimal-but-still-collision-free path
// (never an unsafe one — edgeCost() still returns infinity for occupied
// cells regardless of convergence) on ticks where the deadline is hit
// before the search settles. See docs/PLANNING.md for the benchmarked
// numbers this actually buys.
#ifndef UAV_PLANNING__DSTAR_LITE_PLANNER_HPP_
#define UAV_PLANNING__DSTAR_LITE_PLANNER_HPP_

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#include "uav_planning/grid3d.hpp"

namespace uav_planning
{

class DStarLitePlanner
{
public:
  using Clock = std::chrono::steady_clock;
  static constexpr Clock::time_point kNoDeadline = Clock::time_point::max();

  explicit DStarLitePlanner(size_t max_compute_iterations = 200000);

  // (Re)starts the search around a new goal, taking a full snapshot of
  // `grid`. Call this on mission start and whenever the goal changes.
  void initialize(const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal);

  // Call every tick. Diffs `grid` against the last-seen snapshot, updates
  // only what changed, and returns the current best path from `start` to
  // the goal (world-frame waypoints), or empty if unreachable / not yet
  // initialized / the grid dimensions changed (which forces a fresh
  // initialize() — see docs/PLANNING.md). `deadline`: see the file header
  // comment on the real-time hardening this adds; kNoDeadline (default)
  // preserves the original unbounded-by-time behavior.
  std::vector<Eigen::Vector3d> update(
    const Grid3D & grid, const Eigen::Vector3d & start, Clock::time_point deadline = kNoDeadline);

  // True if the most recent computeShortestPath() (from update() or
  // setLocalizationRisk()) stopped because `deadline` was reached rather
  // than because the search actually converged — i.e., the returned path
  // may be stale/suboptimal this tick, though still collision-free. Pure
  // telemetry/benchmarking signal; does not affect correctness.
  bool lastComputeHitDeadline() const {return last_compute_hit_deadline_;}

  // Margasoochi: confidence-adaptive risk margin (see docs/PLANNING.md,
  // "Algorithmic contribution: Margasoochi"). Feeds the live
  // localization confidence/status (LocalizationState.confidence/status)
  // into the search as a scalar multiplier on each cell's *existing*
  // soft-cost (the proximity buffer Grid3D::inflateObstacles already
  // computed around obstacles) — degrading confidence pushes the search
  // toward paths that keep a wider berth from obstacles, without
  // recomputing the inflation geometry itself.
  //
  // Deliberately quantized into a handful of discrete bands rather than
  // applied continuously: raw covariance/confidence is noisy tick-to-tick,
  // and re-keying every cell in queue_ on every tick would turn D* Lite's
  // bounded incremental update back into an unbounded one. A band only
  // touches `updateVertex()` for cells that actually carry nonzero soft
  // cost (risk_cells_, tracked incrementally as the map changes) — empty
  // free-space cells are untouched, so cost is O(|risk_cells_|) on a band
  // crossing and O(1) otherwise, not O(|grid|).
  void setLocalizationRisk(
    float confidence, uint8_t status, Clock::time_point deadline = kNoDeadline);

  double riskMultiplier() const {return risk_multiplier_;}

  bool isInitialized() const {return initialized_;}

private:
  struct Key
  {
    double k1{std::numeric_limits<double>::infinity()};
    double k2{std::numeric_limits<double>::infinity()};

    bool operator<(const Key & other) const
    {
      if (k1 != other.k1) {return k1 < other.k1;}
      return k2 < other.k2;
    }
    bool operator==(const Key & other) const {return k1 == other.k1 && k2 == other.k2;}
    bool operator!=(const Key & other) const {return !(*this == other);}
  };

  struct QueueEntry
  {
    Key key;
    int id;
  };
  struct QueueEntryGreater
  {
    bool operator()(const QueueEntry & a, const QueueEntry & b) const {return b.key < a.key;}
  };

  void resizeFor(const Grid3D & grid);
  void applySnapshot(const Grid3D & grid);   // full snapshot copy, used by initialize()
  std::vector<int> diffAndUpdateSnapshot(const Grid3D & grid);   // returns changed cell ids
  static int riskBandFor(float confidence, uint8_t status);
  static double riskMultiplierForBand(int band);
  Key calculateKey(int id) const;
  void updateVertex(int id);
  void computeShortestPath(Clock::time_point deadline);
  double edgeCost(int from_id, int to_id) const;
  std::vector<int> neighborIds(int id) const;
  int toId(const GridIndex & idx) const;
  GridIndex toIndex(int id) const;
  double heuristic(int a_id, int b_id) const;

  size_t max_compute_iterations_;
  bool initialized_{false};
  bool last_compute_hit_deadline_{false};

  int size_x_{0};
  int size_y_{0};
  int size_z_{0};
  double resolution_{0.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};

  std::vector<bool> occupied_snapshot_;
  std::vector<double> cost_snapshot_;
  std::vector<int> risk_cells_;   // ids with cost_snapshot_ > 0, kept incrementally
  double risk_multiplier_{1.0};
  int risk_band_{0};

  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<Key> best_key_;     // most recently pushed key per id, for lazy-deletion checks
  std::vector<bool> in_queue_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> queue_;

  int start_id_{-1};
  int goal_id_{-1};
  int last_start_id_{-1};
  double km_{0.0};
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__DSTAR_LITE_PLANNER_HPP_
