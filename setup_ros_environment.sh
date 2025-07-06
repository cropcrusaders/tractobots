#!/bin/bash
# Complete Tractobots Dashboard Launcher
# This script ensures ROS2 and rosbridge are running before launching the dashboard

echo "🚀 Tractobots Dashboard Launcher"
echo "================================"

# Check if ROS2 is installed
echo "📋 Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please install ROS2 Jazzy."
    exit 1
fi

# Source ROS2 environment
echo "⚙️ Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Check if rosbridge is running
echo "🌐 Checking rosbridge server..."
if ! ss -tlnp | grep -q :9090; then
    echo "🚀 Starting rosbridge server..."
    nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &
    echo "⏳ Waiting for rosbridge to start..."
    sleep 5
    
    if ss -tlnp | grep -q :9090; then
        echo "✅ Rosbridge server is running on port 9090"
    else
        echo "❌ Failed to start rosbridge server"
        exit 1
    fi
else
    echo "✅ Rosbridge server is already running on port 9090"
fi

echo "🎯 ROS2 setup complete!"
echo "📊 You can now launch the Windows dashboard."
echo ""
echo "From Windows PowerShell, run:"
echo "  cd c:\\Users\\nicholas\\OneDrive\\Documents\\GitHub\\tractobots"
echo "  powershell.exe -ExecutionPolicy Bypass -File run_dashboard.ps1"
echo ""
echo "Or directly:"
echo "  C:\\Users\\nicholas\\AppData\\Local\\Programs\\Python\\Python313\\python.exe tractobots_win_dashboard.py"
