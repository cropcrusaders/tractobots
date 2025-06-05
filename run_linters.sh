#!/usr/bin/env bash
# Run ament linters for the Tractobots workspace.
# Usage: ./run_linters.sh [<workspace_dir>]
# Default workspace directory is ~/ros2_tractobots

set -e

WS_DIR="${1:-$HOME/ros2_tractobots}"

if ! command -v ament_cpplint >/dev/null 2>&1; then
  echo "ament_cpplint not found. Please install ros-humble-ament-lint-auto and ros-humble-ament-lint-common." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

pushd "$WS_DIR" >/dev/null || { echo "Workspace not found: $WS_DIR" >&2; exit 1; }

ament_cpplint src
ament_uncrustify src
ament_flake8 src

popd >/dev/null

echo "Linters finished successfully."
