#!/usr/bin/env bash
# Runs build + all tests for OUR packages only, skipping the vendored
# fast_lio / livox_ros_driver2 (their upstream code fails our lint suites
# by the hundreds — meaningless noise; see docs/TESTING.md).
#
# Usage: ./scripts/test_ours.sh            (from the repo root)
#        LOW_MEM=1 ./scripts/test_ours.sh  (sequential -j1 build for <4GB RAM machines)
set -euo pipefail
cd "$(dirname "$0")/.."

OURS=(uav_interfaces uav_localization uav_mission uav_planning
  uav_safety uav_vehicle uav_world_model uav_bringup)

BUILD_ARGS=()
if [[ "${LOW_MEM:-0}" == "1" ]]; then
  export MAKEFLAGS="-j1"
  BUILD_ARGS+=(--executor sequential)
fi

colcon build "${BUILD_ARGS[@]}" --packages-select "${OURS[@]}"
colcon test --executor sequential --packages-select "${OURS[@]}"

# Scope the result scan to our packages: a bare `colcon test-result` reads
# ALL of build/, including stale cached results from any earlier full run
# of the vendored packages' test suites.
for pkg in "${OURS[@]}"; do
  if [[ -d "build/${pkg}/test_results" ]]; then
    colcon test-result --test-result-base "build/${pkg}" --verbose
  fi
done

# shellcheck disable=SC1091
source install/setup.bash
pytest tests/contract -v
pytest tests/integration -v
