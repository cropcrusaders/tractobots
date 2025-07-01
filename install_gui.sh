#!/bin/bash
# Install GUI dependencies for Tractobots
# Run this script to set up all GUI interfaces

echo "🎨 Installing Tractobots GUI Dependencies"
echo "========================================"

# Check if we're in the right environment
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS2 not sourced. Attempting to source..."
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        echo "❌ ROS2 not found. Please install ROS2 first."
        exit 1
    }
fi

echo "✅ ROS2 $ROS_DISTRO detected"

# Install Python GUI packages
echo "📦 Installing Python GUI packages..."

# Basic GUI packages (usually already installed)
python3 -m pip install --upgrade pip

# Web Dashboard packages
echo "🌐 Installing Web Dashboard dependencies..."
pip3 install flask flask-socketio python-socketio

# Professional Qt GUI packages  
echo "💎 Installing Qt GUI dependencies..."
pip3 install PyQt5 pyqtgraph numpy

# Data visualization packages
echo "📊 Installing visualization packages..."
pip3 install matplotlib plotly

# Optional packages
echo "🔧 Installing optional packages..."
pip3 install opencv-python Pillow

echo ""
echo "✅ GUI dependencies installed successfully!"
echo ""
echo "🎯 Available GUI interfaces:"
echo "   📱 Simple GUI:      ros2 run tractobots_mission_ui mission_gui_node"
echo "   🎨 Enhanced GUI:    ros2 run tractobots_mission_ui enhanced_gui"
echo "   🌐 Web Dashboard:   ros2 run tractobots_mission_ui web_dashboard"
echo "   💎 Qt Professional: ros2 run tractobots_mission_ui qt_gui"
echo ""
echo "🚀 Or use VS Code tasks:"
echo "   Ctrl+Shift+P → 'Tasks: Run Task' → Choose your GUI"
echo ""
echo "📚 See GUI_GUIDE.md for detailed usage instructions"
