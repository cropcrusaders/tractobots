#!/bin/bash
# Quick build and test script for Tractobots development
# Run this after setting up ROS2 Humble

set -e  # Exit on any error

echo "🚜 Tractobots Development Setup Script"
echo "======================================="

# Check if we're in the right directory
if [ ! -f "README.md" ] || [ ! -d "src" ]; then
    echo "❌ Error: Please run this script from the tractobots repository root"
    exit 1
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 not sourced. Attempting to source Humble..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "❌ ROS2 Humble not found. Please install it first:"
        echo "   ./install_ros2_humble.sh"
        exit 1
    fi
fi

echo "✅ ROS2 $ROS_DISTRO detected"

# Set workspace directory
WS_DIR="$HOME/ros2_tractobots"
echo "📁 Workspace: $WS_DIR"

# Create workspace if it doesn't exist
mkdir -p "$WS_DIR/src"

# Symlink this repo if not already done
if [ ! -L "$WS_DIR/src/tractobots" ]; then
    ln -s "$(pwd)" "$WS_DIR/src/tractobots"
    echo "🔗 Linked tractobots repo to workspace"
fi

cd "$WS_DIR"

echo "📦 Installing dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y || echo "⚠️  Some dependencies may be missing (continuing...)"

echo "🔨 Building workspace..."
colcon build --symlink-install --event-handlers console_direct+

if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
    echo ""
    echo "🚀 To test your system:"
    echo "   source $WS_DIR/install/setup.bash"
    echo "   ros2 run tractobots_bringup system_test"
    echo ""
    echo "� To launch GUI interfaces:"
    echo "   # Enhanced GUI (recommended):"
    echo "   ros2 run tractobots_mission_ui enhanced_gui"
    echo ""
    echo "   # Web Dashboard (mobile-friendly):"
    echo "   ros2 run tractobots_mission_ui web_dashboard"
    echo "   # Then open: http://localhost:5000"
    echo ""
    echo "   # Professional Qt GUI:"
    echo "   ros2 run tractobots_mission_ui qt_gui"
    echo ""
    echo "�🎯 To start the full system:"
    echo "   # Terminal 1:"
    echo "   ros2 launch tractobots_launchers bringup.launch.py"
    echo ""
    echo "   # Terminal 2:"
    echo "   ros2 run joy joy_node"
    echo ""
    echo "   # Terminal 3:"
    echo "   ros2 run tractobots_navigation driver"
    echo ""
    echo "💡 VS Code Users:"
    echo "   Use Ctrl+Shift+P → 'Tasks: Run Task' → Choose your preferred option!"
    echo ""
    echo "🎉 Happy Tractoring!"
else
    echo "❌ Build failed. Check the output above for errors."
    exit 1
fi
