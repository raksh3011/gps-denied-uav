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

#include "uav_planning/theta_star_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <queue>
#include <unordered_map>

namespace uav_planning
{

namespace
{

int64_t flatKey(const GridIndex & idx, int size_x, int size_y)
{
  return static_cast<int64_t>(idx.x) +
         static_cast<int64_t>(idx.y) * size_x +
         static_cast<int64_t>(idx.z) * size_x * size_y;
}

struct OpenEntry
{
  double f;
  double g;   // g at push time, to detect+skip stale entries on pop
  GridIndex idx;
};

struct OpenEntryGreater
{
  bool operator()(const OpenEntry & a, const OpenEntry & b) const {return a.f > b.f;}
};

double heuristic(const GridIndex & a, const GridIndex & b, double resolution)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution;
}

}  // namespace

ThetaStarPlanner::ThetaStarPlanner(size_t max_expansions)
: max_expansions_(max_expansions)
{
}

std::vector<Eigen::Vector3d> ThetaStarPlanner::plan(
  const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal) const
{
  const GridIndex start_idx = grid.worldToIndex(start);
  const GridIndex goal_idx = grid.worldToIndex(goal);

  if (!grid.inBounds(start_idx) || !grid.inBounds(goal_idx)) {return {};}
  if (grid.isOccupied(start_idx) || grid.isOccupied(goal_idx)) {return {};}

  static const std::array<GridIndex, 6> neighbors = {{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
  }};

  const int sx = grid.sizeX();
  const int sy = grid.sizeY();
  const double res = grid.resolution();

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open;
  std::unordered_map<int64_t, double> g_score;
  std::unordered_map<int64_t, GridIndex> parent;

  const int64_t start_key = flatKey(start_idx, sx, sy);
  g_score[start_key] = 0.0;
  parent[start_key] = start_idx;   // parent of start is itself
  open.push({heuristic(start_idx, goal_idx, res), 0.0, start_idx});

  const int64_t goal_key = flatKey(goal_idx, sx, sy);
  size_t expansions = 0;
  bool found = false;

  while (!open.empty() && expansions < max_expansions_) {
    const OpenEntry top = open.top();
    open.pop();
    const GridIndex current = top.idx;
    const int64_t current_key = flatKey(current, sx, sy);

    // Lazy deletion (same pattern as DStarLitePlanner's queue_): a cell
    // can be re-pushed with a better g after this entry was queued, which
    // leaves the OLD entry sitting in the heap. Without this check, that
    // stale entry gets popped later and fully re-expanded — on a large
    // open grid this floods max_expansions with redundant work (each one
    // re-running 6 neighbor traceLine() raycasts) instead of reaching the
    // goal, even though a short path exists.
    if (top.g > g_score.at(current_key) + 1e-9) {continue;}
    ++expansions;

    if (current_key == goal_key) {
      found = true;
      break;
    }

    // Lazy Theta* (Nash, Koenig, Tovey, 2010): unlike eager Theta*, the
    // line-of-sight shortcut is checked ONCE per node actually expanded
    // (here), against that node's own grandparent — not once per
    // candidate neighbor of every node ever pushed. Most pushed
    // candidates are superseded and never expanded, so eager per-neighbor
    // checking wastes the large majority of its raycasts on paths the
    // search never uses; this is what made the eager version blow past
    // its time budget on a large open grid (see docs/PLANNING.md).
    const GridIndex parent_of_current = parent.at(current_key);
    const int64_t parent_key = flatKey(parent_of_current, sx, sy);
    if (parent_key != current_key) {
      const GridIndex grandparent = parent.at(parent_key);
      const int64_t grandparent_key = flatKey(grandparent, sx, sy);
      if (grandparent_key != parent_key) {
        const Eigen::Vector3d grandparent_world = grid.indexToWorld(grandparent);
        const Eigen::Vector3d current_world = grid.indexToWorld(current);
        const Grid3D::LineTrace trace = grid.traceLine(grandparent_world, current_world);
        if (trace.clear) {
          const double candidate_g = g_score.at(grandparent_key) +
            (current_world - grandparent_world).norm() + trace.cost;
          if (candidate_g < g_score.at(current_key) - 1e-9) {
            g_score[current_key] = candidate_g;
            parent[current_key] = grandparent;
          }
        }
      }
    }

    for (const auto & step : neighbors) {
      const GridIndex next{current.x + step.x, current.y + step.y, current.z + step.z};
      if (!grid.inBounds(next) || grid.isOccupied(next)) {continue;}

      const int64_t next_key = flatKey(next, sx, sy);
      // Plain grid-adjacent relaxation — no line-of-sight check here.
      // `current`'s own g already reflects any grandparent shortcut from
      // above, so the any-angle benefit still propagates to `next`; it's
      // just resolved lazily instead of eagerly.
      const double tentative_g = g_score.at(current_key) + res + grid.traversalCost(next);

      const auto it = g_score.find(next_key);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[next_key] = tentative_g;
        parent[next_key] = current;
        open.push({tentative_g + heuristic(next, goal_idx, res), tentative_g, next});
      }
    }
  }

  if (!found) {return {};}

  std::vector<Eigen::Vector3d> path;
  GridIndex walk = goal_idx;
  path.push_back(grid.indexToWorld(walk));
  int64_t walk_key = flatKey(walk, sx, sy);
  while (walk_key != start_key) {
    walk = parent.at(walk_key);
    walk_key = flatKey(walk, sx, sy);
    path.push_back(grid.indexToWorld(walk));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace uav_planning
