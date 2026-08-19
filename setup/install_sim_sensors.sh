#!/usr/bin/env bash
# Patches the locally-downloaded PX4 `x500_base` Gazebo model with our
# LiDAR + IMU sensor payload (simulation/models/x500_lidar/sensors.sdf.xml).
#
# WHY x500_base, not x500: PX4 spawns the vehicle as "x500", but that model
# is assembled dynamically in memory at spawn time (confirmed from Gazebo's
# own warnings referencing it as `<data-string>`, not a file path) — built
# from x500_base plus PX4's own plugin/motor config. Patching x500/model.sdf
# on disk does nothing; it's not what's actually read at spawn time.
# x500_base IS read from disk on every spawn (confirmed via a real file path
# in its own warnings), so patching it is what actually propagates through.
#
# WHY patch the stock model at all instead of shipping our own "x500_lidar"
# model: PX4's SITL launch plumbing (Ninja `gz_<model>` targets,
# PX4_SIM_MODEL airframe auto-detection) is hardcoded to a fixed, small set
# of known model names — we spent a long debugging session confirming
# there's no supported way to add a new model name without either finding
# an undocumented Ninja target-generation mechanism or force-writing a
# persisted PX4 parameter. Patching x500_base sidesteps all of that: every
# standard PX4 command (`make px4_sitl gz_x500`) just works unchanged.
# See docs/LOCALIZATION.md for the full story.
#
# Idempotent — safe to re-run. Re-running after sensors.sdf.xml changes
# re-applies the current version (restores from backup first).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SNIPPET="$REPO_ROOT/simulation/models/x500_lidar/sensors.sdf.xml"
MODEL_SDF="$HOME/.simulation-gazebo/models/x500_base/model.sdf"
BACKUP="$HOME/.simulation-gazebo/models/x500_base/model.sdf.orig"

if [ ! -f "$MODEL_SDF" ]; then
  echo "ERROR: $MODEL_SDF not found." >&2
  echo "Run 'python3 \$HOME/PX4-Autopilot/Tools/simulation/gz/simulation-gazebo' once" >&2
  echo "first (Ctrl+C it once the Gazebo window opens) to download the stock models." >&2
  exit 1
fi

if [ ! -f "$SNIPPET" ]; then
  echo "ERROR: sensor snippet not found at $SNIPPET" >&2
  exit 1
fi

# Always restore from a pristine backup first, so re-running after an edit
# to sensors.sdf.xml doesn't stack duplicate copies or patch an
# already-patched file.
if [ -f "$BACKUP" ]; then
  cp "$BACKUP" "$MODEL_SDF"
else
  cp "$MODEL_SDF" "$BACKUP"
fi

if ! grep -q "</model>" "$MODEL_SDF"; then
  echo "ERROR: no </model> closing tag found in $MODEL_SDF — unexpected file format." >&2
  exit 1
fi

last_model_close_line=$(grep -n "</model>" "$MODEL_SDF" | tail -1 | cut -d: -f1)
insert_after_line=$((last_model_close_line - 1))

# Extract only the marked BEGIN..END region from the snippet (not the
# explanatory header comment above it) and insert it right before the
# model's closing tag.
awk '/<!-- BEGIN gps-denied-uav sensors -->/,/<!-- END gps-denied-uav sensors -->/' "$SNIPPET" \
  > /tmp/gps-denied-uav-sensors-fragment.xml

sed -i "${insert_after_line}r /tmp/gps-denied-uav-sensors-fragment.xml" "$MODEL_SDF"
rm -f /tmp/gps-denied-uav-sensors-fragment.xml

echo "Patched $MODEL_SDF with lidar_link + imu_link (backup at $BACKUP)."
echo "Verify with: grep -c 'gps-denied-uav sensors' \"$MODEL_SDF\"   # expect 2 (BEGIN + END)"
