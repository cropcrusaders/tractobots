#!/bin/bash
# Comprehensive Tractobots Test Script
# This script sources ROS2 and runs Python tests

echo "🚜 TRACTOBOTS COMPREHENSIVE SYSTEM TEST"
echo "======================================"

# Source ROS2
echo "📦 Sourcing ROS2..."
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f "/home/bass/ros2_tractobots/install/setup.bash" ]; then
    echo "📦 Sourcing workspace..."
    source /home/bass/ros2_tractobots/install/setup.bash
fi

echo "✅ ROS_DISTRO: $ROS_DISTRO"

# Run the Python test
echo "🧪 Running Python comprehensive test..."
cd /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots
python3 comprehensive_test.py
