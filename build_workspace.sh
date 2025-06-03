#!/usr/bin/env bash
# Simple helper to build the Tractobots workspace.
# Usage: ./build_workspace.sh [<workspace_dir>]
# Default workspace directory is ~/ros2_tractobots

set -e

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
colcon build --symlink-install

echo "\nBuild complete. Source the workspace with:\n  source \"$WS_DIR/install/setup.bash\""
