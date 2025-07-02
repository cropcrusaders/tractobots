#!/usr/bin/env bash
# Tractobots Desktop Application Launcher
# Launches the PyQt5 desktop app with proper environment setup

echo "🚜 Launching Tractobots Desktop Application..."

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set up environment
export PYTHONPATH="$SCRIPT_DIR/src/tractobots_mission_ui:$PYTHONPATH"

# Check dependencies
echo "🔍 Checking dependencies..."

# Check PyQt5
if ! python3 -c "import PyQt5" 2>/dev/null; then
    echo "❌ PyQt5 not found. Installing..."
    sudo apt install -y python3-pyqt5 python3-pyqt5.qtwidgets
fi

# Check GeoPandas
if ! python3 -c "import geopandas" 2>/dev/null; then
    echo "❌ GeoPandas not found. Installing..."
    sudo apt install -y python3-geopandas python3-shapely python3-pyproj python3-fiona
fi

# Check if ROS2 is available
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 available"
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
    
    # Source workspace if available
    if [ -f "$HOME/ros2_tractobots/install/setup.bash" ]; then
        source "$HOME/ros2_tractobots/install/setup.bash"
        echo "✅ ROS2 workspace sourced"
    fi
else
    echo "⚠️  ROS2 not available (some features will be limited)"
fi

# Set display for WSL
if grep -q microsoft /proc/version; then
    echo "🐧 WSL detected - setting up display"
    export DISPLAY=:0
fi

echo "🚀 Starting Tractobots Desktop App..."
cd "$SCRIPT_DIR"

# Launch the application
python3 tractobots_desktop_app.py

echo "👋 Tractobots Desktop App closed"
