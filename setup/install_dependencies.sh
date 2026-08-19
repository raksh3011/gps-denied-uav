#!/usr/bin/env bash
# Installs all system dependencies for the GPS-denied UAV stack.
# Run INSIDE WSL2 Ubuntu 24.04. Do not run on native Windows.
set -euo pipefail

if ! grep -qi microsoft /proc/version; then
  echo "WARNING: this does not look like WSL2. Continuing anyway." >&2
fi

source "$(dirname "$0")/versions.txt" 2>/dev/null || true

echo "== Updating apt =="
sudo apt update && sudo apt upgrade -y

echo "== Base build tools =="
sudo apt install -y \
  build-essential cmake git curl wget gnupg lsb-release \
  software-properties-common python3-pip python3-venv python3-dev \
  libeigen3-dev libpcl-dev pcl-tools

echo "== Locale (required by ROS 2) =="
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "== ROS 2 Jazzy apt repo =="
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "== ROS 2 Jazzy desktop + dev tools =="
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep

echo "== Gazebo (bundled harmonic via ros-jazzy) =="
sudo apt install -y ros-jazzy-ros-gz

echo "== Extra ROS 2 packages used by this stack =="
sudo apt install -y \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-eigen \
  ros-jazzy-pcl-conversions ros-jazzy-pcl-ros \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-nav-msgs \
  ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ament-cmake

echo "== rosdep init/update =="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "== PX4 SITL (source build for jMAVSim/Gazebo integration) =="
if [ ! -d "$HOME/PX4-Autopilot" ]; then
  git clone --branch "${PX4_FIRMWARE_TAG:-v1.15.0}" --recursive \
    https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot"
fi
bash "$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx

echo "== Python tooling env =="
python3 -m pip install --user --upgrade pip
python3 -m pip install --user pytest numpy scipy pymavlink

echo "== Done. Run setup/verify_environment.sh next. =="
