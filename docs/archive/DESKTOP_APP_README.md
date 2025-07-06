# 🚜 Tractobots Desktop Application

## Overview

The Tractobots Desktop Application is a modern, user-friendly PyQt5-based GUI for managing autonomous tractor field operations. It provides a comprehensive interface for shapefile processing, field boundary management, and coverage path planning.

## Features

### 🗺️ Field Management
- **Shapefile Import**: Load .shp files from your operation center
- **Field Visualization**: View and manage field boundaries
- **Field Details**: Inspect field properties (name, crop type, area)
- **Data Export**: Export field data to YAML/JSON formats

### 🛣️ Path Planning
- **Coverage Path Generation**: Create waypoint paths for field coverage
- **Configurable Parameters**: Adjust swath width, overlap, and patterns
- **Path Statistics**: View distance, time estimates, and waypoint counts
- **Multiple Export Formats**: Export to ROS2 YAML, CSV waypoints

### 🤖 ROS2 Integration
- **System Status**: Real-time status of ROS2 and dependencies
- **Service Launch**: Start ROS2 field boundary services
- **Workspace Integration**: Seamless integration with ROS2 workspace

### 🔧 System Tools
- **Dependency Checking**: Automatic verification of required packages
- **System Diagnostics**: Built-in troubleshooting tools
- **Documentation Access**: Quick access to user guides

## Installation & Setup

### Prerequisites
```bash
# Install PyQt5 and dependencies
sudo apt install python3-pyqt5 python3-pyqt5.qtwidgets

# Install shapefile processing dependencies
sudo apt install python3-geopandas python3-shapely python3-pyproj python3-fiona python3-yaml

# Optional: ROS2 for full integration
sudo apt install ros-jazzy-desktop  # or ros-humble-desktop
```

### Quick Start
```bash
# Method 1: Use the launcher script (recommended)
./launch_desktop_app.sh

# Method 2: Direct launch
python3 tractobots_desktop_app.py

# Method 3: ROS2 integration (after building workspace)
cd ~/ros2_tractobots
source install/setup.bash
ros2 run tractobots_mission_ui desktop_app
```

## Usage Guide

### 1. Loading Shapefiles
1. Launch the desktop app
2. Go to the **Field Management** tab
3. Click **Browse...** to select your .shp file
4. Click **Load Shapefile** to import field boundaries
5. Review loaded fields in the table

### 2. Generating Coverage Paths
1. Switch to the **Path Planning** tab
2. Select a field from the dropdown
3. Configure path parameters:
   - **Swath Width**: Equipment width (typically 4-8 meters)
   - **Overlap**: Safety overlap between passes
   - **Pattern**: Coverage pattern (Parallel, Spiral, Zigzag)
4. Click **Generate Coverage Path**
5. Review path statistics and waypoints

### 3. Exporting Data
- **Field Data**: Export to YAML/JSON for ROS2 navigation
- **Coverage Paths**: Export complete path data with metadata
- **Waypoints**: Export simple CSV for GPS navigation systems

### 4. ROS2 Integration
- **System Status**: Check ROS2 availability in the header
- **Launch Services**: Use Tools → Launch ROS2 Components
- **Real-time Data**: Connect to live GPS and mission data (when available)

## Application Architecture

### Main Components
```
TractorbotsDesktopApp (QMainWindow)
├── StatusWidget (System status display)
├── FieldManagementTab (Shapefile loading and field management)
│   ├── File loading controls
│   ├── Field data table
│   └── Export functionality
└── PathPlanningTab (Coverage path generation)
    ├── Path planning parameters
    ├── Path generation controls
    └── Waypoint management
```

### Worker Threads
- **FieldDataWorker**: Background shapefile processing
- **PathPlanningWorker**: Background path generation
- **Non-blocking UI**: Responsive interface during long operations

### Data Flow
```
Shapefile (.shp) → FieldDataWorker → Field Table → PathPlanningWorker → Coverage Path → Export
```

## Customization

### Styling
The app uses modern PyQt5 styling with:
- Professional color scheme (blue accents)
- Rounded buttons and group boxes
- Hover effects and visual feedback
- Responsive layout design

### Settings Persistence
- Window geometry and state saved automatically
- User preferences stored in system settings
- Automatic restoration on next launch

### Extension Points
- Additional tabs can be added for new features
- Worker threads for background processing
- Plugin-style architecture for new file formats

## Troubleshooting

### Common Issues

**"PyQt5 not found"**
```bash
sudo apt install python3-pyqt5 python3-pyqt5.qtwidgets
```

**"GeoPandas not found"**
```bash
sudo apt install python3-geopandas python3-shapely python3-pyproj python3-fiona
```

**"Display not available" (WSL)**
```bash
# Install X11 server on Windows (VcXsrv or Xming)
export DISPLAY=:0
```

**"ROS2 not available"**
```bash
# Install ROS2 Jazzy (recommended for Ubuntu 24.04)
./install_ros2_jazzy_noble.sh
```

### Debug Mode
```bash
# Launch with debug output
python3 tractobots_desktop_app.py --debug

# Check system status
python3 diagnose_problems.py
```

## Development

### Code Structure
- **tractobots_desktop_app.py**: Main application file
- **launch_desktop_app.sh**: Launcher script with dependency checking
- **tractobots.desktop**: Desktop entry for system integration

### Dependencies
- **PyQt5**: GUI framework
- **GeoPandas**: Shapefile processing
- **ROS2** (optional): Robot integration
- **Threading**: Background processing

### Future Enhancements
- Real-time GPS tracking display
- 3D field visualization
- Mission execution monitoring
- Multi-field operation planning
- Weather integration
- Equipment status monitoring

## License

This desktop application is part of the Tractobots project and follows the same license terms.
