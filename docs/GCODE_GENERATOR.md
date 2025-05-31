# G-code Generator Guide

This document explains how to create simple G-code waypoint files and how to build the Tractobots workspace so the `gcode_reader_node` can consume them.

## 1. Overview

The repository contains a small G-code parser located in `src/Gcode_parser`. It expects G-code with GPS coordinates and optional implement commands. A generator script can produce such files from a list of waypoints.

## 2. Example Generator Script

The following Python snippet converts a list of latitude/longitude pairs to G-code commands:

```python
waypoints = [
    (35.6895, -120.6910),
    (35.6896, -120.6915),
]

with open("path.gcode", "w") as f:
    for lat, lon in waypoints:
        f.write(f"G1 X{lat:.6f} Y{lon:.6f}\n")
```

`G1` indicates a move. The `X` and `Y` values are interpreted as latitude and longitude by the parser. Additional commands such as `M100`/`M101` can drop or lift a plow.

## 3. Building the Repository

1. Install ROS 2 Humble and required tools:
   ```bash
   ./install_ros2_humble.sh
   source /opt/ros/humble/setup.bash
   ```
2. Create a workspace and clone the repository:
   ```bash
   mkdir -p ~/ros2_tractobots/src
   cd ~/ros2_tractobots/src
   git clone <this repository>
   cd ..
   colcon build --symlink-install
   source install/setup.bash
   ```

## 4. Running the G-code Reader

Place your generated `path.gcode` somewhere on disk. Launch the reader node with parameters pointing at the file and the latitude/longitude of your origin:

```bash
ros2 run gcode_parser gcode_reader_node \
  --ros-args -p gcode_file:=/full/path/to/path.gcode \
  -p origin_lat:=35.6894 -p origin_lon:=-120.6912
```

The node publishes a `nav_msgs/Path` on `gcode_path` and sends hitch commands based on the G-code.

