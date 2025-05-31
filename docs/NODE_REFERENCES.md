# ROS 2 Node Reference

This document lists the main ROS 2 nodes provided by the Tractobots repository and where their source code lives. Each node is built automatically when the workspace is compiled with `colcon`.

The recommended build steps are:

```bash
mkdir -p ~/ros2_tractobots/src
cd ~/ros2_tractobots/src
# clone this repository
cd ..
colcon build --symlink-install
```

Sourcing `install/setup.bash` after the build makes all nodes available.

## Node Locations

| Node Name | Package / Path | Notes |
|-----------|----------------|-------|
| `iso_bus_watchdog_node` | `src/Iso_bus_watchdog/src/iso_bus_watchdog_node.cpp` | C++ node using AgIsoStack++ for monitoring ISOBUS engine parameters. |
| `gcode_reader_node` | `src/Gcode_parser/Src/gcode_reader_node.cpp` | Parses GPS waypoints from G-code files. |
| `AdNav_Node` | `src/Advancednavigation/Src/advanced_navigation_driver.cpp` | Publishes IMU and NavSat data from Advanced Navigation devices. |
| `NV08C_node` | `src/tractobots_gps/tractobots_gps/NV08C_node.py` | Python node for NV08C GPS receivers. |
| `driver` | `src/tractobots_navigation/tractobots_navigation/driver.py` | Main tele‑op and line‑following driver. |
| `gps_parser`, `imu_publisher`, `pose_transformer` | `src/tractobots_robot_localization/tractobots_robot_localization` | Helper nodes for robot localization. |
| `mission_ui_node`, `mission_gui_node` | `src/tractobots_mission_ui/tractobots_mission_ui` | Web and Tk GUI interfaces for mission control. |

All packages use standard `ament_cmake` or `ament_python` build tooling and do not require any private dependencies. After running `colcon build` you can launch individual nodes with `ros2 run <package> <executable>`.

