#!/usr/bin/env bash
# Creates/builds the colcon workspace. Idempotent — safe to re-run.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

echo "== Sourcing ROS 2 Jazzy =="
# ROS 2's setup.bash references some variables before assigning them,
# which trips `set -u`. Relax it only for the sourcing itself.
set +u
source /opt/ros/jazzy/setup.bash
set -u

echo "== rosdep install for src/ =="
rosdep install --from-paths src --ignore-src -r -y

echo "== colcon build =="
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "== Adding auto-source to ~/.bashrc (once) =="
MARKER="# >>> gps-denied-uav workspace >>>"
if ! grep -qF "$MARKER" "$HOME/.bashrc" 2>/dev/null; then
  {
    echo "$MARKER"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "source \"$REPO_ROOT/install/setup.bash\""
    echo "# <<< gps-denied-uav workspace <<<"
  } >> "$HOME/.bashrc"
  echo "Added workspace sourcing to ~/.bashrc. Open a new shell or 'source ~/.bashrc'."
fi

echo "== Workspace built. Source it with: =="
echo "   source $REPO_ROOT/install/setup.bash"
