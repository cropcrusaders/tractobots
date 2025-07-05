# 🚜 Tractobots Live Dashboard System

## Overview

Your Tractobots system now includes **multiple graphical interfaces** for real-time monitoring and control:

### 📊 Available Dashboards

1. **🌐 Web Dashboard** (Recommended)
   - Works everywhere - no display required
   - Access via web browser at `http://localhost:5000`
   - Real-time updates every 2 seconds
   - Control buttons for ROS2, Gazebo, and system management

2. **🎨 GUI Dashboard**
   - Professional desktop application
   - Live matplotlib plots and charts
   - Real-time field visualization
   - Requires display environment

3. **💻 Terminal Dashboard**
   - Text-based interface
   - Works in any terminal
   - Live system status and data
   - No GUI dependencies required

## 🚀 Quick Start

### Option 1: Use the Launcher (Recommended)
```bash
# Run the interactive launcher
./launch_tractobots_dashboard.sh

# Choose your preferred dashboard type
# 1 = Web Dashboard (works everywhere)
# 2 = GUI Dashboard (requires display)
# 3 = Terminal Dashboard (text-based)
# 4 = System test
```

### Option 2: Direct Launch
```bash
# Web Dashboard
python3 tractobots_web_dashboard.py

# GUI Dashboard
python3 tractobots_live_dashboard.py

# Terminal Dashboard
python3 tractobots_terminal_dashboard.py
```

### Option 3: Check System Status
```bash
# Check what's available
python3 check_dashboard_status.py
```

## 📋 Dashboard Features

### 🌐 Web Dashboard Features
- **Real-time System Status**: ROS2, Gazebo, Navigation, GPS
- **Live Tractor Data**: Position, speed, heading, battery
- **System Statistics**: CPU, memory, disk usage
- **Control Panel**: Start/stop ROS2, Gazebo, and other systems
- **Activity Logs**: Real-time system messages
- **Responsive Design**: Works on desktop and mobile

### 🎨 GUI Dashboard Features
- **Live Plots**: 4 real-time matplotlib charts
- **Position Tracking**: 2D tractor movement visualization
- **Speed Monitoring**: Real-time speed graphs
- **Battery Status**: Battery level over time
- **System Resources**: CPU, memory, disk, network usage
- **Dark Theme**: Professional appearance
- **Control Buttons**: System management controls

### 💻 Terminal Dashboard Features
- **System Status**: Color-coded status indicators
- **Live Data**: Real-time tractor information
- **System Stats**: CPU, memory, disk usage
- **Updates**: Refreshes every 2 seconds
- **Keyboard Controls**: Interactive commands
- **Logs**: Real-time system messages

## 🔧 Installation & Dependencies

### Required Dependencies
```bash
# Core Python packages
pip3 install flask matplotlib numpy

# For GUI dashboard (if using desktop environment)
sudo apt-get install python3-tk

# Optional: Enhanced features
pip3 install plotly opencv-python Pillow
```

### Check Installation
```bash
# Run system check
python3 check_dashboard_status.py

# This will show what's available and what needs installation
```

## 🎯 Usage Examples

### Web Dashboard
```bash
# Start web dashboard
python3 tractobots_web_dashboard.py

# Open browser to: http://localhost:5000
# Dashboard shows:
# - System status (ROS2, Gazebo, etc.)
# - Live tractor data (position, speed, battery)
# - Control buttons (start/stop systems)
# - Real-time logs
```

### GUI Dashboard
```bash
# Start GUI dashboard
python3 tractobots_live_dashboard.py

# Features:
# - 4 live matplotlib plots
# - Position tracking
# - Speed monitoring
# - Battery status
# - System resource usage
```

### Terminal Dashboard
```bash
# Start terminal dashboard
python3 tractobots_terminal_dashboard.py

# Shows:
# - Color-coded system status
# - Live tractor data
# - System statistics
# - Real-time updates
# - Keyboard controls
```

## 🎮 Dashboard Controls

### Web Dashboard Controls
- **Start ROS2**: Launch ROS2 system
- **Start Gazebo**: Launch Gazebo simulation
- **Stop All**: Stop all systems
- **Auto-refresh**: Updates every 2 seconds

### GUI Dashboard Controls
- **System Status**: Live status indicators
- **Control Buttons**: Start/stop various systems
- **Plot Controls**: Zoom, pan, and examine data
- **Window Controls**: Resize and move plots

### Terminal Dashboard Controls
- **q**: Quit dashboard
- **r**: Restart ROS2
- **g**: Start Gazebo
- **s**: Stop all systems
- **Ctrl+C**: Exit

## 📊 What You Can Monitor

### System Status
- ✅ **ROS2**: Node count, system health
- ✅ **Gazebo**: Simulation status
- ✅ **Navigation**: Path planning, obstacle avoidance
- ✅ **GPS**: Location accuracy, signal strength
- ✅ **Simulation**: 3D environment status

### Live Tractor Data
- 📍 **Position**: X, Y coordinates in real-time
- 🏃 **Speed**: Current velocity in m/s
- 🧭 **Heading**: Direction in degrees
- 🔋 **Battery**: Power level percentage
- 📡 **GPS Status**: Lock status and accuracy

### System Resources
- 💻 **CPU Usage**: Processing load
- 🧠 **Memory**: RAM usage
- 💾 **Disk**: Storage usage
- 🌐 **Network**: Activity monitoring

## 🔧 Troubleshooting

### Web Dashboard Issues
```bash
# If web dashboard won't start
pip3 install flask
python3 tractobots_web_dashboard.py

# If can't access browser
# Check: http://localhost:5000
# Try: http://127.0.0.1:5000
```

### GUI Dashboard Issues
```bash
# If GUI won't start
sudo apt-get install python3-tk
pip3 install matplotlib numpy

# If display issues
export DISPLAY=:0
python3 tractobots_live_dashboard.py
```

### Terminal Dashboard Issues
```bash
# If terminal dashboard has issues
python3 tractobots_terminal_dashboard.py

# Should work in any terminal environment
```

## 🌟 Advanced Features

### Real-time Updates
- All dashboards update automatically
- Web dashboard: 2-second intervals
- GUI dashboard: 1-second intervals
- Terminal dashboard: 2-second intervals

### Remote Access
- Web dashboard can be accessed remotely
- Change host in code to '0.0.0.0' for network access
- Access from other devices via IP address

### Customization
- Modify update intervals in code
- Change colors and themes
- Add custom charts and data sources
- Extend with additional system monitoring

## 🎉 You Now Have

✅ **3 Different Dashboard Options**
✅ **Real-time System Monitoring**
✅ **Live Tractor Data Visualization**
✅ **System Control Capabilities**
✅ **Professional Interface Design**
✅ **Cross-platform Compatibility**

## 🚀 Next Steps

1. **Choose your preferred dashboard** using the launcher
2. **Monitor your system** in real-time
3. **Control ROS2 and Gazebo** via dashboard buttons
4. **Import your field data** for visualization
5. **Extend with custom features** as needed

**Your graphical monitoring system is ready!** 🎯
