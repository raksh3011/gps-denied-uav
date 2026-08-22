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

// compare_planners: INDEPENDENT, NON-FUNCTIONAL analysis tool. See
// analysis/README.md before reading this file — it explains why this
// exists, the honest baseline definition, and exactly how "does not
// influence runtime path planning" is guaranteed (short version: this
// file only ever CALLS the existing public API of uav_planning_core and
// times/measures the results externally; it never modifies, subclasses,
// or reaches into any planner's internals, and nothing here is invoked
// by any launch file, node, or test).
//
// Not built by default with the rest of uav_planning — see this
// package's CMakeLists.txt for the opt-in gate.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>
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

// Not std::numbers::pi (C++20 has it, but M_PI is a non-standard glibc
// extension not guaranteed by <cmath> — this codebase already avoids it,
// see px4_command_bridge.cpp's identical kPi constant).
constexpr double kPi = 3.14159265358979323846;

double ms(Clock::duration d)
{
  return std::chrono::duration<double, std::milli>(d).count();
}

template<typename Fn>
double timeMsOf(Fn && fn)
{
  const auto t0 = Clock::now();
  fn();
  return ms(Clock::now() - t0);
}

// ---- Pure geometry helpers over a returned path — no planner internals
// touched, everything computed from the same waypoint list the caller
// already gets back from the public plan()/update() API. ----

double pathLength(const std::vector<Eigen::Vector3d> & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += (path[i] - path[i - 1]).norm();
  }
  return len;
}

// Sum of heading-change angles between consecutive segments, in degrees.
// Lower = smoother. A perfectly straight line scores 0.
double totalTurningAngleDeg(const std::vector<Eigen::Vector3d> & path)
{
  double total = 0.0;
  for (size_t i = 2; i < path.size(); ++i) {
    const Eigen::Vector3d a = (path[i - 1] - path[i - 2]);
    const Eigen::Vector3d b = (path[i] - path[i - 1]);
    if (a.norm() < 1e-9 || b.norm() < 1e-9) {continue;}
    const double cos_angle = std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0);
    total += std::acos(cos_angle) * 180.0 / kPi;
  }
  return total;
}

// Number of heading changes exceeding `threshold_deg` — a coarser,
// more intuitive companion to the summed-angle metric above.
int numTurns(const std::vector<Eigen::Vector3d> & path, double threshold_deg)
{
  int count = 0;
  for (size_t i = 2; i < path.size(); ++i) {
    const Eigen::Vector3d a = (path[i - 1] - path[i - 2]);
    const Eigen::Vector3d b = (path[i] - path[i - 1]);
    if (a.norm() < 1e-9 || b.norm() < 1e-9) {continue;}
    const double cos_angle = std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0);
    const double angle_deg = std::acos(cos_angle) * 180.0 / kPi;
    if (angle_deg > threshold_deg) {++count;}
  }
  return count;
}

// Minimum surface clearance from any obstacle along the WHOLE path,
// sampling each segment (not just waypoints) so a close pass between
// two sparse any-angle waypoints isn't missed.
double minClearance(
  const std::vector<Eigen::Vector3d> & path, const std::vector<ObstacleSphere> & obstacles,
  double step_m = 0.1)
{
  if (obstacles.empty() || path.empty()) {return std::numeric_limits<double>::infinity();}
  double worst = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d & a = path[i - 1];
    const Eigen::Vector3d & b = path[i];
    const double seg_len = (b - a).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(seg_len / step_m)));
    for (int s = 0; s <= steps; ++s) {
      const Eigen::Vector3d p = a + (static_cast<double>(s) / steps) * (b - a);
      for (const auto & obstacle : obstacles) {
        worst = std::min(worst, (p - obstacle.center).norm() - obstacle.radius);
      }
    }
  }
  return worst;
}

Grid3D makeGrid()
{
  return Grid3D(0.25, Eigen::Vector3d(-15.0, -7.5, -5.0), 120, 60, 40);
}

const Eigen::Vector3d kStart(-10.0, 0.0, 1.5);
const Eigen::Vector3d kGoal(10.0, 0.0, 3.0);

// The same 8-pillar layout used by demo_mission.launch.xml and the
// ThetaStarPlanner regression test — not a new, cherry-picked field.
std::vector<ObstacleSphere> sparseObstacles()
{
  return {
    {Eigen::Vector3d(-6.0, 1.5, 1.25), 0.5},
    {Eigen::Vector3d(-6.0, -3.5, 1.25), 0.5},
    {Eigen::Vector3d(-2.0, -1.0, 1.25), 0.6},
    {Eigen::Vector3d(-2.0, 4.0, 1.25), 0.4},
    {Eigen::Vector3d(2.0, 1.0, 1.25), 0.5},
    {Eigen::Vector3d(2.0, -4.0, 1.25), 0.4},
    {Eigen::Vector3d(6.0, -1.5, 1.25), 0.5},
    {Eigen::Vector3d(6.0, 3.0, 1.25), 0.5},
  };
}

// A harder, denser field within the same arena bounds.
std::vector<ObstacleSphere> denseObstacles()
{
  std::vector<ObstacleSphere> obstacles;
  for (int i = -8; i <= 8; i += 2) {
    for (int j = -1; j <= 1; ++j) {
      obstacles.push_back({Eigen::Vector3d(i, j * 2.0, 1.25 + 0.3 * j), 0.45});
    }
  }
  return obstacles;
}

constexpr double kHardMarginM = 0.3;
constexpr double kSoftMarginM = 1.5;
constexpr double kSoftCostWeight = 5.0;

// ---- Result record for one scenario ----
struct MetricSet
{
  double path_length_m{-1.0};
  double planning_time_ms{-1.0};
  double avg_replan_time_ms{-1.0};   // -1 = not applicable to this scenario
  double turning_deg{-1.0};
  int num_turns{-1};
  double min_clearance_m{-1.0};
  double dynamic_response_ms{-1.0};   // -1 = not applicable
  std::vector<Eigen::Vector3d> path;
};

struct ScenarioResult
{
  std::string name;
  double straight_line_m{0.0};
  MetricSet traditional;   // plain A*, full replan
  MetricSet theta;         // our global reference, Lazy Theta*
  MetricSet dstar;         // our flown path, D* Lite + Margasoochi
  bool has_margasoochi_test{false};
  double margasoochi_margin_widening_m{-1.0};   // traditional: always N/A, see README
};

MetricSet runAstarOnce(
  const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const std::vector<ObstacleSphere> & obstacles)
{
  MetricSet m;
  AStarPlanner astar;
  m.planning_time_ms = timeMsOf([&]() {m.path = astar.plan(grid, start, goal);});
  m.path_length_m = pathLength(m.path);
  m.turning_deg = totalTurningAngleDeg(m.path);
  m.num_turns = numTurns(m.path, 5.0);
  m.min_clearance_m = minClearance(m.path, obstacles);
  return m;
}

MetricSet runThetaOnce(
  const Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const std::vector<ObstacleSphere> & obstacles)
{
  MetricSet m;
  ThetaStarPlanner theta;
  m.planning_time_ms = timeMsOf([&]() {m.path = theta.plan(grid, start, goal);});
  m.path_length_m = pathLength(m.path);
  m.turning_deg = totalTurningAngleDeg(m.path);
  m.num_turns = numTurns(m.path, 5.0);
  m.min_clearance_m = minClearance(m.path, obstacles);
  return m;
}

// Runs D* Lite for `ticks` steps of the vehicle crawling toward goal
// (worst-case-latency style, matching bench_planning's methodology),
// returns the LAST tick's path plus the average per-tick update() cost.
MetricSet runDStarSteady(
  Grid3D & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  const std::vector<ObstacleSphere> & obstacles, int ticks = 30)
{
  MetricSet m;
  DStarLitePlanner dstar;
  m.planning_time_ms = timeMsOf([&]() {dstar.initialize(grid, start, goal);});

  double total_ms = 0.0;
  Eigen::Vector3d pos = start;
  const Eigen::Vector3d step = (goal - start).normalized() * 0.1;
  for (int i = 0; i < ticks; ++i) {
    pos += step;
    total_ms += timeMsOf([&]() {m.path = dstar.update(grid, pos);});
  }
  m.avg_replan_time_ms = total_ms / ticks;
  m.path_length_m = pathLength(m.path);
  m.turning_deg = totalTurningAngleDeg(m.path);
  m.num_turns = numTurns(m.path, 5.0);
  m.min_clearance_m = minClearance(m.path, obstacles);
  return m;
}

ScenarioResult runScenario(
  const std::string & name, std::vector<ObstacleSphere> obstacles)
{
  ScenarioResult result;
  result.name = name;
  result.straight_line_m = (kGoal - kStart).norm();

  Grid3D grid_a = makeGrid();
  grid_a.inflateObstacles(obstacles, kHardMarginM, kSoftMarginM, kSoftCostWeight);
  result.traditional = runAstarOnce(grid_a, kStart, kGoal, obstacles);

  Grid3D grid_b = makeGrid();
  grid_b.inflateObstacles(obstacles, kHardMarginM, kSoftMarginM, kSoftCostWeight);
  result.theta = runThetaOnce(grid_b, kStart, kGoal, obstacles);

  Grid3D grid_c = makeGrid();
  grid_c.inflateObstacles(obstacles, kHardMarginM, kSoftMarginM, kSoftCostWeight);
  result.dstar = runDStarSteady(grid_c, kStart, kGoal, obstacles);

  return result;
}

// Scenario 4: an obstacle appears mid-flight, directly ahead on the
// current path. Measures re-plan latency and post-reroute clearance for
// both approaches, on equal footing (same trigger point, same new
// obstacle).
ScenarioResult runDynamicObstacleScenario()
{
  ScenarioResult result;
  result.name = "Dynamic obstacle appears mid-flight";
  result.straight_line_m = (kGoal - kStart).norm();
  auto obstacles = sparseObstacles();

  const Eigen::Vector3d midpoint = kStart + (kGoal - kStart) * 0.5;
  const Eigen::Vector3d direction = (kGoal - kStart).normalized();
  // 2m AHEAD of the trigger point along the direction of travel — not
  // co-located with the vehicle's own position at the moment the
  // obstacle appears. An obstacle placed on top of the vehicle would
  // make D* Lite correctly (but uselessly for this test) report the
  // start cell itself as occupied, which tests nothing about rerouting.
  const ObstacleSphere new_obstacle{midpoint + direction * 2.0, 0.6};

  // Traditional: fly (conceptually) to the midpoint, then a new
  // obstacle appears; measure the cost and quality of a full re-plan
  // from scratch, which is all a non-incremental planner can do.
  {
    Grid3D grid = makeGrid();
    grid.inflateObstacles(obstacles, kHardMarginM, kSoftMarginM, kSoftCostWeight);
    AStarPlanner astar;
    const auto before = astar.plan(grid, kStart, kGoal);
    (void)before;

    obstacles.push_back(new_obstacle);
    grid.inflateObstacles({new_obstacle}, kHardMarginM, kSoftMarginM, kSoftCostWeight);

    MetricSet m;
    m.dynamic_response_ms = timeMsOf([&]() {m.path = astar.plan(grid, kStart, kGoal);});
    m.path_length_m = pathLength(m.path);
    m.min_clearance_m = minClearance(m.path, obstacles);
    result.traditional = m;
    obstacles.pop_back();
  }

  // Ours: D* Lite has already been running (steady-state, as in
  // production); the SAME new obstacle appears and we measure only the
  // one tick's incremental update() cost, not a full re-plan.
  {
    Grid3D grid = makeGrid();
    grid.inflateObstacles(obstacles, kHardMarginM, kSoftMarginM, kSoftCostWeight);
    DStarLitePlanner dstar;
    dstar.initialize(grid, kStart, kGoal);
    // Advance a few ticks first, matching a vehicle already mid-flight.
    Eigen::Vector3d pos = kStart;
    const Eigen::Vector3d step = (midpoint - kStart) / 10.0;
    for (int i = 0; i < 10; ++i) {
      pos += step;
      dstar.update(grid, pos);
    }

    obstacles.push_back(new_obstacle);
    grid.inflateObstacles({new_obstacle}, kHardMarginM, kSoftMarginM, kSoftCostWeight);

    MetricSet m;
    m.dynamic_response_ms = timeMsOf([&]() {m.path = dstar.update(grid, pos);});
    m.path_length_m = pathLength(m.path);
    m.min_clearance_m = minClearance(m.path, obstacles);
    result.dstar = m;
  }

  return result;
}

// Scenario 5: localization confidence degrades mid-flight. Only
// meaningful for our stack (Margasoochi) — plain A*/D* Lite have no
// concept of localization confidence at all, so "traditional" is
// reported as N/A, not 0.
ScenarioResult runMargasoochiScenario()
{
  ScenarioResult result;
  result.name = "Localization confidence degrades mid-flight";
  result.straight_line_m = (kGoal - kStart).norm();
  result.has_margasoochi_test = true;

  // A goal placed close alongside one obstacle's soft-margin zone, so
  // there's an actual live trade-off for the risk multiplier to act on
  // — a path that goes nowhere near any soft-cost zone would show zero
  // effect regardless of how the multiplier is set.
  const Eigen::Vector3d start(-4.0, 0.0, 1.25);
  const Eigen::Vector3d goal(0.0, 0.0, 1.25);
  const std::vector<ObstacleSphere> obstacles = {{Eigen::Vector3d(-2.0, 0.0, 1.25), 0.6}};

  Grid3D grid = makeGrid();
  grid.inflateObstacles(obstacles, kHardMarginM, kSoftMarginM, kSoftCostWeight);

  DStarLitePlanner dstar;
  dstar.initialize(grid, start, goal);
  dstar.setLocalizationRisk(0.95F, 0 /* STATUS_NOMINAL */);
  const auto nominal_path = dstar.update(grid, start);
  const double nominal_clearance = minClearance(nominal_path, obstacles);

  dstar.setLocalizationRisk(0.2F, 2 /* STATUS_LOST */);
  const auto degraded_path = dstar.update(grid, start);
  const double degraded_clearance = minClearance(degraded_path, obstacles);

  result.margasoochi_margin_widening_m = degraded_clearance - nominal_clearance;

  result.dstar.path = degraded_path;
  result.dstar.path_length_m = pathLength(degraded_path);
  result.dstar.min_clearance_m = degraded_clearance;
  result.traditional.min_clearance_m = nominal_clearance;   // no adaptive response possible
  return result;
}

// ---- Minimal hand-rolled JSON writer — no external dependency, this
// tool has no need for a full JSON library for a flat, known structure.
// JSON has no Infinity/NaN literal (e.g. min_clearance_m is +infinity
// for the open-field scenario, which has no obstacle to measure against)
// — write those as `null` rather than emitting invalid JSON.
void writeNumber(std::ofstream & out, double value)
{
  if (std::isfinite(value)) {
    out << value;
  } else {
    out << "null";
  }
}

void writePath(std::ofstream & out, const std::vector<Eigen::Vector3d> & path)
{
  out << "[";
  for (size_t i = 0; i < path.size(); ++i) {
    out << "[" << path[i].x() << "," << path[i].y() << "," << path[i].z() << "]";
    if (i + 1 < path.size()) {out << ",";}
  }
  out << "]";
}

void writeMetricSet(std::ofstream & out, const MetricSet & m)
{
  out << "{\"path_length_m\":";
  writeNumber(out, m.path_length_m);
  out << ",\"planning_time_ms\":";
  writeNumber(out, m.planning_time_ms);
  out << ",\"avg_replan_time_ms\":";
  writeNumber(out, m.avg_replan_time_ms);
  out << ",\"turning_deg\":";
  writeNumber(out, m.turning_deg);
  out << ",\"num_turns\":" << m.num_turns
      << ",\"min_clearance_m\":";
  writeNumber(out, m.min_clearance_m);
  out << ",\"dynamic_response_ms\":";
  writeNumber(out, m.dynamic_response_ms);
  out << ",\"path\":";
  writePath(out, m.path);
  out << "}";
}

void writeScenario(std::ofstream & out, const ScenarioResult & r)
{
  out << "{"
      << "\"name\":\"" << r.name << "\","
      << "\"straight_line_m\":";
  writeNumber(out, r.straight_line_m);
  out << ",\"traditional\":";
  writeMetricSet(out, r.traditional);
  out << ",\"theta\":";
  writeMetricSet(out, r.theta);
  out << ",\"dstar\":";
  writeMetricSet(out, r.dstar);
  out << ",\"has_margasoochi_test\":" << (r.has_margasoochi_test ? "true" : "false")
      << ",\"margasoochi_margin_widening_m\":";
  writeNumber(out, r.margasoochi_margin_widening_m);
  out << "}";
}

void printSummary(const ScenarioResult & r)
{
  std::printf("\n=== %s ===\n", r.name.c_str());
  std::printf("  straight-line distance: %.2f m\n", r.straight_line_m);
  std::printf(
    "  Traditional (A*, full replan): length=%.2fm  plan=%.3fms  replan=%.3fms  "
    "turns=%d (%.1fdeg)  clearance=%.2fm  dyn_response=%.3fms\n",
    r.traditional.path_length_m, r.traditional.planning_time_ms,
    r.traditional.avg_replan_time_ms, r.traditional.num_turns, r.traditional.turning_deg,
    r.traditional.min_clearance_m, r.traditional.dynamic_response_ms);
  std::printf(
    "  Ours - global ref (Lazy Theta*): length=%.2fm  plan=%.3fms  "
    "turns=%d (%.1fdeg)  clearance=%.2fm\n",
    r.theta.path_length_m, r.theta.planning_time_ms, r.theta.num_turns,
    r.theta.turning_deg, r.theta.min_clearance_m);
  std::printf(
    "  Ours - flown (D* Lite+Margasoochi): length=%.2fm  init=%.3fms  replan=%.3fms  "
    "turns=%d (%.1fdeg)  clearance=%.2fm  dyn_response=%.3fms\n",
    r.dstar.path_length_m, r.dstar.planning_time_ms, r.dstar.avg_replan_time_ms,
    r.dstar.num_turns, r.dstar.turning_deg, r.dstar.min_clearance_m,
    r.dstar.dynamic_response_ms);
  if (r.has_margasoochi_test) {
    std::printf(
      "  Margasoochi margin widening under degraded localization: %.3fm "
      "(traditional: N/A, no adaptive mechanism)\n",
      r.margasoochi_margin_widening_m);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string out_path = argc > 1 ? argv[1] : "analysis/results/results.json";

  std::vector<ScenarioResult> results;
  results.push_back(runScenario("Open field", {}));
  results.push_back(runScenario("Sparse obstacles (project demo arena)", sparseObstacles()));
  results.push_back(runScenario("Dense/cluttered obstacles", denseObstacles()));
  results.push_back(runDynamicObstacleScenario());
  results.push_back(runMargasoochiScenario());

  for (const auto & r : results) {
    printSummary(r);
  }

  std::ofstream out(out_path);
  if (!out.is_open()) {
    std::printf("\nCould not open '%s' for writing — pass a writable path as argv[1].\n",
      out_path.c_str());
    return 1;
  }
  out << "{\"scenarios\":[";
  for (size_t i = 0; i < results.size(); ++i) {
    writeScenario(out, results[i]);
    if (i + 1 < results.size()) {out << ",";}
  }
  out << "]}";
  out.close();
  std::printf("\nWrote %s\n", out_path.c_str());
  return 0;
}
