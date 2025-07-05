#!/bin/bash

# Tractobots Dashboard Launcher
# This script provides multiple ways to launch the dashboard

clear
echo "🚜 TRACTOBOTS DASHBOARD LAUNCHER"
echo "================================"
echo ""
echo "Available Dashboard Options:"
echo "1. Web Dashboard    (Recommended - works everywhere)"
echo "2. GUI Dashboard    (Requires display)"
echo "3. Terminal Dashboard (Text-based)"
echo "4. Simple Test     (Check if system works)"
echo ""

# Function to launch web dashboard
launch_web_dashboard() {
    echo "🚀 Starting Web Dashboard..."
    echo "Installing dependencies..."
    pip3 install flask > /dev/null 2>&1
    
    echo "🌐 Launching web server..."
    python3 tractobots_web_dashboard.py &
    WEB_PID=$!
    
    sleep 3
    
    echo "✅ Web Dashboard is running!"
    echo "📱 Open your browser and go to:"
    echo "   http://localhost:5000"
    echo ""
    echo "🎯 Features available:"
    echo "   • Real-time system status"
    echo "   • Live tractor data"
    echo "   • System control buttons"
    echo "   • Activity logs"
    echo ""
    echo "Press Ctrl+C to stop the dashboard"
    
    # Wait for user to stop
    trap "echo ''; echo '🛑 Stopping web dashboard...'; kill $WEB_PID 2>/dev/null; exit 0" INT
    while kill -0 $WEB_PID 2>/dev/null; do
        sleep 1
    done
}

# Function to launch GUI dashboard
launch_gui_dashboard() {
    echo "🚀 Starting GUI Dashboard..."
    
    # Check if display is available
    if [[ -z "$DISPLAY" ]]; then
        echo "❌ No display available"
        echo "   GUI dashboard requires a graphical environment"
        echo "   Try the web dashboard instead (option 1)"
        return 1
    fi
    
    # Check dependencies
    echo "📦 Checking dependencies..."
    python3 -c "import tkinter" 2>/dev/null || {
        echo "Installing tkinter..."
        sudo apt-get update && sudo apt-get install -y python3-tk
    }
    
    python3 -c "import matplotlib" 2>/dev/null || {
        echo "Installing matplotlib..."
        pip3 install matplotlib
    }
    
    echo "🎨 Launching GUI..."
    python3 tractobots_live_dashboard.py
}

# Function to launch terminal dashboard
launch_terminal_dashboard() {
    echo "🚀 Starting Terminal Dashboard..."
    python3 tractobots_terminal_dashboard.py
}

# Function to run simple test
run_simple_test() {
    echo "🧪 Running Simple Test..."
    python3 -c "
import sys
print('✅ Python is working')
print(f'   Version: {sys.version}')

try:
    import subprocess
    result = subprocess.run(['ros2', '--version'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        print('✅ ROS2 is available')
    else:
        print('❌ ROS2 not found')
except:
    print('❌ ROS2 not available')

try:
    import tkinter
    print('✅ GUI support available')
except:
    print('❌ GUI support not available')

try:
    import flask
    print('✅ Flask available for web dashboard')
except:
    print('❌ Flask not available (run: pip3 install flask)')

print('')
print('🎯 System ready for dashboard launch!')
"
}

# Main menu
while true; do
    echo "Choose an option (1-4):"
    read -p "Enter your choice: " choice
    
    case $choice in
        1)
            launch_web_dashboard
            break
            ;;
        2)
            launch_gui_dashboard
            break
            ;;
        3)
            launch_terminal_dashboard
            break
            ;;
        4)
            run_simple_test
            echo ""
            echo "Press Enter to return to menu..."
            read
            ;;
        *)
            echo "❌ Invalid choice. Please enter 1, 2, 3, or 4."
            ;;
    esac
done
