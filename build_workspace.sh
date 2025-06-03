#!/usr/bin/env bash
# Simple helper to build the Tractobots workspace.
# Usage: ./build_workspace.sh [<workspace_dir>]
# Default workspace directory is ~/ros2_tractobots

set -e
set -o pipefail  # ensure failure inside pipelines is caught

WS_DIR="${1:-$HOME/ros2_tractobots}"
REPO_ROOT="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"

mkdir -p "$WS_DIR/src"

# Symlink this repository into the workspace if not already present
if [ ! -e "$WS_DIR/src/tractobots" ]; then
    ln -s "$REPO_ROOT" "$WS_DIR/src/tractobots"
fi

cd "$WS_DIR"

if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon not found. Please install ROS 2 Humble before running this script." >&2
    exit 1
fi

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install \
  --event-handlers console_cohesion+ --event-handlers console_direct+ \
  2>&1 | tee build.log
BUILD_STATUS=${PIPESTATUS[0]}

if [ $BUILD_STATUS -ne 0 ]; then
  echo "\nBuild failed. Last 20 lines of build.log:" >&2
  tail -n 20 build.log >&2
  # Also show tails of individual package logs if available
  FAILED_PKGS=$(grep -E "^(Failed|Aborted)\s+<<<" build.log | awk '{print $2}')
  for pkg in $FAILED_PKGS; do
    LOG_DIR="log/latest_build/$pkg"
    if [ -d "$LOG_DIR" ]; then
      for f in "$LOG_DIR"/*.log; do
        [ -f "$f" ] || continue
        echo "--- ${pkg}/$(basename "$f") ---" >&2
        tail -n 20 "$f" >&2
      done
    fi
  done
  echo "---" >&2
  echo "Full log at: $WS_DIR/build.log" >&2
  exit $BUILD_STATUS
fi

echo "\nBuild complete. Source the workspace with:\n  source \"$WS_DIR/install/setup.bash\""
