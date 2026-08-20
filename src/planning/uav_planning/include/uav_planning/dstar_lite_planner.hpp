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
#ifndef UAV_PLANNING__DSTAR_LITE_PLANNER_HPP_
#define UAV_PLANNING__DSTAR_LITE_PLANNER_HPP_

#include <Eigen/Core>

#include <limits>
#include <queue>
#include <vector>

#include "uav_planning/grid3d.hpp"

namespace uav_planning
{

class DStarLitePlanner
{
public:
  explicit DStarLitePlanner(size_t max_compute_iterations = 200000);

  // (Re)starts the search around a new goal, taking a full snapshot of
  // `grid`. Call this on mission start and whenever the goal changes.
  void initialize(const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal);

  // Call every tick. Diffs `grid` against the last-seen snapshot, updates
  // only what changed, and returns the current best path from `start` to
  // the goal (world-frame waypoints), or empty if unreachable / not yet
  // initialized / the grid dimensions changed (which forces a fresh
  // initialize() — see docs/PLANNING.md).
  std::vector<Eigen::Vector3d> update(const Grid3D & grid, const Eigen::Vector3d & start);

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
  Key calculateKey(int id) const;
  void updateVertex(int id);
  void computeShortestPath();
  double edgeCost(int from_id, int to_id) const;
  std::vector<int> neighborIds(int id) const;
  int toId(const GridIndex & idx) const;
  GridIndex toIndex(int id) const;
  double heuristic(int a_id, int b_id) const;

  size_t max_compute_iterations_;
  bool initialized_{false};

  int size_x_{0};
  int size_y_{0};
  int size_z_{0};
  double resolution_{0.0};
  Eigen::Vector3d origin_{Eigen::Vector3d::Zero()};

  std::vector<bool> occupied_snapshot_;
  std::vector<double> cost_snapshot_;

  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<Key> best_key_;     // most recently pushed key per id, for lazy-deletion staleness checks
  std::vector<bool> in_queue_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> queue_;

  int start_id_{-1};
  int goal_id_{-1};
  int last_start_id_{-1};
  double km_{0.0};
};

}  // namespace uav_planning

#endif  // UAV_PLANNING__DSTAR_LITE_PLANNER_HPP_
