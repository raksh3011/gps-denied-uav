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

#include "uav_planning/dstar_lite_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace uav_planning
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kOriginToleranceM = 1e-6;   // see the rolling-map caveat in docs/PLANNING.md
}  // namespace

DStarLitePlanner::DStarLitePlanner(size_t max_compute_iterations)
: max_compute_iterations_(max_compute_iterations)
{
}

int DStarLitePlanner::toId(const GridIndex & idx) const
{
  return idx.x + idx.y * size_x_ + idx.z * size_x_ * size_y_;
}

GridIndex DStarLitePlanner::toIndex(int id) const
{
  const int x = id % size_x_;
  const int y = (id / size_x_) % size_y_;
  const int z = id / (size_x_ * size_y_);
  return GridIndex{x, y, z};
}

std::vector<int> DStarLitePlanner::neighborIds(int id) const
{
  static const std::array<GridIndex, 6> steps = {{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
  }};
  const GridIndex idx = toIndex(id);
  std::vector<int> out;
  out.reserve(6);
  for (const auto & step : steps) {
    const int nx = idx.x + step.x;
    const int ny = idx.y + step.y;
    const int nz = idx.z + step.z;
    if (nx < 0 || nx >= size_x_ || ny < 0 || ny >= size_y_ || nz < 0 || nz >= size_z_) {continue;}
    out.push_back(toId(GridIndex{nx, ny, nz}));
  }
  return out;
}

double DStarLitePlanner::edgeCost(int /*from_id*/, int to_id) const
{
  if (occupied_snapshot_[to_id]) {return kInf;}
  return resolution_ + cost_snapshot_[to_id];
}

double DStarLitePlanner::heuristic(int a_id, int b_id) const
{
  const GridIndex a = toIndex(a_id);
  const GridIndex b = toIndex(b_id);
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) * resolution_;
}

DStarLitePlanner::Key DStarLitePlanner::calculateKey(int id) const
{
  const double m = std::min(g_[id], rhs_[id]);
  return Key{m + heuristic(start_id_, id) + km_, m};
}

void DStarLitePlanner::updateVertex(int id)
{
  if (id != goal_id_) {
    double best = kInf;
    for (int n : neighborIds(id)) {
      const double c = edgeCost(id, n) + g_[n];
      if (c < best) {best = c;}
    }
    rhs_[id] = best;
  }

  if (g_[id] != rhs_[id]) {
    best_key_[id] = calculateKey(id);
    in_queue_[id] = true;
    queue_.push({best_key_[id], id});
  } else {
    in_queue_[id] = false;
  }
}

void DStarLitePlanner::computeShortestPath()
{
  size_t iterations = 0;
  while (iterations < max_compute_iterations_) {
    // Discard stale duplicate entries (lazy deletion) until we find the
    // real current top, or the queue is empty.
    while (!queue_.empty()) {
      const QueueEntry top = queue_.top();
      if (!in_queue_[top.id] || !(top.key == best_key_[top.id])) {
        queue_.pop();
        continue;
      }
      break;
    }
    if (queue_.empty()) {break;}

    const QueueEntry top = queue_.top();
    const Key start_key = calculateKey(start_id_);
    if (!(top.key < start_key) && rhs_[start_id_] == g_[start_id_]) {
      break;   // s_start is locally consistent and nothing more promising remains
    }

    queue_.pop();
    ++iterations;
    const int u = top.id;
    const Key k_new = calculateKey(u);

    if (top.key < k_new) {
      best_key_[u] = k_new;
      in_queue_[u] = true;
      queue_.push({k_new, u});
    } else if (g_[u] > rhs_[u]) {
      g_[u] = rhs_[u];
      in_queue_[u] = false;
      for (int s : neighborIds(u)) {updateVertex(s);}
    } else {
      // g_[u] <= rhs_[u]: u got (or stayed) worse. Conservative but
      // definitely-correct handling: invalidate u and let updateVertex
      // recompute rhs for u and everything that could depend on it from
      // scratch, rather than the textbook's more surgical "only if
      // rhs(s) == c(s,u) + g_old" check — costs a little more work per
      // update, not a correctness gap.
      g_[u] = kInf;
      updateVertex(u);
      for (int s : neighborIds(u)) {updateVertex(s);}
    }
  }
}

void DStarLitePlanner::resizeFor(const Grid3D & grid)
{
  size_x_ = grid.sizeX();
  size_y_ = grid.sizeY();
  size_z_ = grid.sizeZ();
  resolution_ = grid.resolution();
  origin_ = grid.indexToWorld(GridIndex{0, 0, 0}) -
    Eigen::Vector3d(0.5 * resolution_, 0.5 * resolution_, 0.5 * resolution_);

  const size_t n = static_cast<size_t>(size_x_) * size_y_ * size_z_;
  occupied_snapshot_.assign(n, false);
  cost_snapshot_.assign(n, 0.0);
  g_.assign(n, kInf);
  rhs_.assign(n, kInf);
  best_key_.assign(n, Key{});
  in_queue_.assign(n, false);
  queue_ = std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>();
}

void DStarLitePlanner::applySnapshot(const Grid3D & grid)
{
  for (int z = 0; z < size_z_; ++z) {
    for (int y = 0; y < size_y_; ++y) {
      for (int x = 0; x < size_x_; ++x) {
        const GridIndex idx{x, y, z};
        const int id = toId(idx);
        occupied_snapshot_[id] = grid.isOccupied(idx);
        cost_snapshot_[id] = grid.traversalCost(idx);
      }
    }
  }
}

std::vector<int> DStarLitePlanner::diffAndUpdateSnapshot(const Grid3D & grid)
{
  std::vector<int> changed;
  for (int z = 0; z < size_z_; ++z) {
    for (int y = 0; y < size_y_; ++y) {
      for (int x = 0; x < size_x_; ++x) {
        const GridIndex idx{x, y, z};
        const int id = toId(idx);
        const bool occ = grid.isOccupied(idx);
        const double cost = grid.traversalCost(idx);
        if (occ != occupied_snapshot_[id] || cost != cost_snapshot_[id]) {
          occupied_snapshot_[id] = occ;
          cost_snapshot_[id] = cost;
          changed.push_back(id);
        }
      }
    }
  }
  return changed;
}

void DStarLitePlanner::initialize(
  const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal)
{
  resizeFor(grid);
  applySnapshot(grid);

  const GridIndex start_idx = grid.worldToIndex(start);
  const GridIndex goal_idx = grid.worldToIndex(goal);

  if (!grid.inBounds(start_idx) || !grid.inBounds(goal_idx)) {
    initialized_ = false;
    return;
  }

  start_id_ = toId(start_idx);
  last_start_id_ = start_id_;
  goal_id_ = toId(goal_idx);
  km_ = 0.0;

  if (!grid.isOccupied(goal_idx)) {
    rhs_[goal_id_] = 0.0;
    best_key_[goal_id_] = calculateKey(goal_id_);
    in_queue_[goal_id_] = true;
    queue_.push({best_key_[goal_id_], goal_id_});
  }

  initialized_ = true;
  computeShortestPath();
}

std::vector<Eigen::Vector3d> DStarLitePlanner::update(
  const Grid3D & grid, const Eigen::Vector3d & start)
{
  if (!initialized_) {return {};}

  // A rolling/re-centering local map, or a resolution/size change, moves
  // what each integer GridIndex means underneath us — our snapshot would
  // silently refer to the wrong world locations. Detect it and force the
  // caller to initialize() fresh rather than continue incrementally on
  // stale assumptions. See docs/PLANNING.md's rolling-map caveat.
  const Eigen::Vector3d new_origin = grid.indexToWorld(GridIndex{0, 0, 0}) -
    Eigen::Vector3d(0.5 * grid.resolution(), 0.5 * grid.resolution(), 0.5 * grid.resolution());
  if (grid.sizeX() != size_x_ || grid.sizeY() != size_y_ || grid.sizeZ() != size_z_ ||
    std::abs(grid.resolution() - resolution_) > 1e-9 ||
    (new_origin - origin_).norm() > kOriginToleranceM)
  {
    initialized_ = false;
    return {};
  }

  const GridIndex start_idx = grid.worldToIndex(start);
  if (!grid.inBounds(start_idx)) {return {};}
  const int new_start_id = toId(start_idx);

  if (new_start_id != start_id_) {
    km_ += heuristic(last_start_id_, start_id_);
    last_start_id_ = start_id_;
    start_id_ = new_start_id;
  }

  const std::vector<int> changed = diffAndUpdateSnapshot(grid);
  for (int id : changed) {
    updateVertex(id);
    for (int n : neighborIds(id)) {updateVertex(n);}
  }

  computeShortestPath();

  if (occupied_snapshot_[start_id_] || std::isinf(g_[start_id_])) {return {};}

  std::vector<Eigen::Vector3d> path;
  int current = start_id_;
  path.push_back(grid.indexToWorld(toIndex(current)));

  const int max_steps = size_x_ * size_y_ * size_z_ + 1;
  int steps = 0;
  while (current != goal_id_ && steps < max_steps) {
    int best_n = -1;
    double best_cost = kInf;
    for (int n : neighborIds(current)) {
      const double c = edgeCost(current, n) + g_[n];
      if (c < best_cost) {
        best_cost = c;
        best_n = n;
      }
    }
    if (best_n < 0 || std::isinf(best_cost)) {return {};}
    current = best_n;
    path.push_back(grid.indexToWorld(toIndex(current)));
    ++steps;
  }
  if (current != goal_id_) {return {};}
  return path;
}

}  // namespace uav_planning
