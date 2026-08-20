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

#include "uav_planning/astar_planner.hpp"

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

AStarPlanner::AStarPlanner(size_t max_expansions)
: max_expansions_(max_expansions)
{
}

std::vector<Eigen::Vector3d> AStarPlanner::plan(
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
  std::unordered_map<int64_t, GridIndex> came_from;

  const int64_t start_key = flatKey(start_idx, sx, sy);
  g_score[start_key] = 0.0;
  open.push({heuristic(start_idx, goal_idx, res), start_idx});

  const int64_t goal_key = flatKey(goal_idx, sx, sy);
  size_t expansions = 0;
  bool found = false;

  while (!open.empty() && expansions < max_expansions_) {
    const GridIndex current = open.top().idx;
    open.pop();
    ++expansions;

    const int64_t current_key = flatKey(current, sx, sy);
    if (current_key == goal_key) {
      found = true;
      break;
    }

    for (const auto & step : neighbors) {
      const GridIndex next{current.x + step.x, current.y + step.y, current.z + step.z};
      if (!grid.inBounds(next) || grid.isOccupied(next)) {continue;}

      const int64_t next_key = flatKey(next, sx, sy);
      const double step_cost = res + grid.traversalCost(next);
      const double tentative_g = g_score[current_key] + step_cost;

      const auto it = g_score.find(next_key);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[next_key] = tentative_g;
        came_from[next_key] = current;
        open.push({tentative_g + heuristic(next, goal_idx, res), next});
      }
    }
  }

  if (!found) {return {};}

  std::vector<Eigen::Vector3d> path;
  GridIndex walk = goal_idx;
  path.push_back(grid.indexToWorld(walk));
  int64_t walk_key = flatKey(walk, sx, sy);
  while (walk_key != start_key) {
    walk = came_from.at(walk_key);
    walk_key = flatKey(walk, sx, sy);
    path.push_back(grid.indexToWorld(walk));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace uav_planning
