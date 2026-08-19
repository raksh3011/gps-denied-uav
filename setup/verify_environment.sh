#!/usr/bin/env bash
# Verifies every dependency required by this project. Prints PASS/FAIL per item.
# Exit code 0 if everything passes, 1 otherwise.
set -uo pipefail

PASS=0
FAIL=0

check() {
  local name="$1"; shift
  if "$@" >/dev/null 2>&1; then
    printf "PASS  %s\n" "$name"
    PASS=$((PASS+1))
  else
    printf "FAIL  %s\n" "$name"
    FAIL=$((FAIL+1))
  fi
}

check_min_version() {
  local name="$1" have="$2" want="$3"
  if [ -z "$have" ]; then
    printf "FAIL  %s (not found)\n" "$name"; FAIL=$((FAIL+1)); return
  fi
  if printf '%s\n%s\n' "$want" "$have" | sort -V -C; then
    printf "PASS  %s (%s >= %s)\n" "$name" "$have" "$want"; PASS=$((PASS+1))
  else
    printf "FAIL  %s (%s < %s)\n" "$name" "$have" "$want"; FAIL=$((FAIL+1))
  fi
}

echo "===== Environment Verification ====="

# WSL2
check "Running under WSL2" bash -c "grep -qi microsoft /proc/version"

# Ubuntu version
UBUNTU_VER=$(lsb_release -rs 2>/dev/null || echo "")
check_min_version "Ubuntu >= 24.04" "$UBUNTU_VER" "24.04"

# Compiler
GCC_VER=$(gcc -dumpversion 2>/dev/null || echo "")
check_min_version "GCC >= 13" "$GCC_VER" "13"

# CMake
CMAKE_VER=$(cmake --version 2>/dev/null | head -1 | awk '{print $3}')
check_min_version "CMake >= 3.22" "$CMAKE_VER" "3.22"

# Python
PY_VER=$(python3 --version 2>/dev/null | awk '{print $2}')
check_min_version "Python >= 3.12" "$PY_VER" "3.12"

# Git
check "Git installed" bash -c "command -v git"

# colcon
check "colcon installed" bash -c "command -v colcon"

# ROS 2 Jazzy
check "ROS 2 Jazzy setup.bash present" bash -c "[ -f /opt/ros/jazzy/setup.bash ]"
check "ROS 2 env sourced (ROS_DISTRO=jazzy)" bash -c "source /opt/ros/jazzy/setup.bash && [ \"\$ROS_DISTRO\" = 'jazzy' ]"
check "ros2 CLI works" bash -c "source /opt/ros/jazzy/setup.bash && ros2 --help"

# rosdep
check "rosdep initialized" bash -c "[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]"

# Gazebo (ros_gz)
check "ros-jazzy-ros-gz installed" dpkg -s ros-jazzy-ros-gz

# Eigen
check "Eigen3 headers present" bash -c "[ -d /usr/include/eigen3 ]"

# PCL
check "PCL headers present" bash -c "pkg-config --exists pcl_common-1.14 || dpkg -s libpcl-dev"

# PX4
check "PX4-Autopilot cloned" bash -c "[ -d \"$HOME/PX4-Autopilot\" ]"

# Python packages
check "pytest importable" python3 -c "import pytest"
check "numpy importable" python3 -c "import numpy"

echo "====================================="
echo "PASS: $PASS   FAIL: $FAIL"

if [ "$FAIL" -eq 0 ]; then
  echo "Environment OK."
  exit 0
else
  echo "Environment incomplete. Re-run setup/install_dependencies.sh or fix items above."
  exit 1
fi
