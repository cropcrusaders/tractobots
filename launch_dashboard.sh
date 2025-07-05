#!/bin/bash
# Tractobots Dashboard Launcher - Updated
# This script will launch the instant dashboard

echo "🚜 TRACTOBOTS DASHBOARD LAUNCHER"
echo "================================"
echo ""
echo "🎯 Starting instant web dashboard..."
echo ""

# Launch the instant dashboard
python3 instant_dashboard.py
    echo "❌ Dashboard file not found in current directory"
    echo "Please run this script from the tractobots root directory"
    exit 1
fi

# Check Python
echo "🔍 Checking Python..."
if command -v python3 &> /dev/null; then
    echo "✅ Python3 found: $(python3 --version)"
else
    echo "❌ Python3 not found"
    exit 1
fi

# Check required modules
echo "🔍 Checking required modules..."
python3 -c "import tkinter; print('✅ tkinter available')" 2>/dev/null || {
    echo "❌ tkinter not available"
    echo "Installing tkinter..."
    sudo apt-get update && sudo apt-get install -y python3-tk
}

python3 -c "import matplotlib; print('✅ matplotlib available')" 2>/dev/null || {
    echo "❌ matplotlib not available"
    echo "Installing matplotlib..."
    pip3 install matplotlib
}

python3 -c "import numpy; print('✅ numpy available')" 2>/dev/null || {
    echo "❌ numpy not available"
    echo "Installing numpy..."
    pip3 install numpy
}

echo ""
echo "🚀 Starting Tractobots Live Dashboard..."
echo "==========================================="
echo ""

# Check if display is available
if [[ -z "$DISPLAY" ]]; then
    echo "⚠️  Warning: No DISPLAY environment variable found"
    echo "   This might be a headless environment"
    echo "   The GUI might not work properly"
fi

# Launch the dashboard
echo "🎯 Launching dashboard..."
python3 tractobots_live_dashboard.py

echo ""
echo "🛑 Dashboard closed"
