# CLAUDE.md - Tractobots Development Guide

## Project Overview

Tractobots is an autonomous agriculture platform (v2.1.0) built on **ROS 2 Jazzy** for precision farming. It combines a ROS 2 robotics backend with a Windows PyQt5 dashboard connected via WebSocket (rosbridge). The system handles autonomous row-by-row guidance, mission planning, ISOBUS implement control, and high-precision GPS/INS navigation.

- **Maintainer**: Nicholas Bass (nicholasbass@crop-crusaders.com)
- **License**: GPLv3
- **Python**: 3.10+
- **Platforms**: Linux (Ubuntu 24.04), Windows (via WSL2)

## Repository Structure

```
tractobots/
├── src/                          # ROS 2 workspace (17 packages)
│   ├── tractobots_bringup/       # System launch & configuration
│   ├── tractobots_core/          # E-stop, diagnostics, core services (C++)
│   ├── tractobots_vehicle/       # CAN/J1939 interface abstraction (C++)
│   ├── tractobots_planning/      # Fields2Cover path planning wrapper (C++)
│   ├── tractobots_control/       # Steering & implement control algorithms (C++)
│   ├── tractobots_navigation/    # Driver/teleoperation node (Python)
│   ├── tractobots_bridges/       # AgOpenGPS & NMEA GPS bridges (C++)
│   ├── tractobots_gps/           # GPS receiver interface (Python)
│   ├── tractobots_robot_localization/  # EKF, NavSat, pose transforms (Python)
│   ├── tractobots_nav2/          # Navigation2 stack configuration
│   ├── tractobots_description/   # URDF robot model & TF tree
│   ├── tractobots_launchers/     # Top-level launch files
│   ├── tractobots_mission_ui/    # Web & Tkinter mission control (Python)
│   ├── Advancednavigation/       # Advanced Navigation INS driver (C++)
│   ├── Gcode_parser/             # G-code mission file parsing (C++)
│   ├── iso_bus_watchdog/         # ISOBUS CAN bus monitoring (C++)
│   └── agisostack_plus_plus/    # AgIsoStack vendor library (C++)
├── dashboard/                    # Windows PyQt5 GUI application
│   ├── dashboard.py              # Main dashboard (~1075 lines)
│   ├── utils/                    # WSL port forwarding utilities
│   └── tests/                    # Dashboard tests
├── Arduino-JD6330/               # John Deere implement firmware (Arduino)
├── Arduino-MT765/                # Massey Ferguson implement firmware (Arduino)
├── field_data/                   # Sample field boundaries (shapefiles)
├── docs/                         # Extended documentation
├── .github/                      # CI/CD workflows & actions
├── .devcontainer/                # Dev container configuration
├── cfg/                          # Code style configs (uncrustify)
└── tools/                        # Utility scripts
```

## Build System

### ROS 2 Workspace (colcon)

The project uses **colcon** as its build system with **ament_cmake** for C++ packages and **ament_python** for pure Python packages.

**Build the workspace:**
```bash
./build_workspace.sh [<workspace_dir>]
# Default workspace: ~/ros2_tractobots
```

This script:
1. Creates a workspace and symlinks the repo into `src/`
2. Runs `rosdep install` to fetch dependencies
3. Runs `colcon build --symlink-install`

**Manual build:**
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --event-handlers console_direct+
```

**CMake presets** (in `CMakePresets.json`):
- `debug` - Debug build
- `relwithdebinfo` - Release with debug info

### Windows Dashboard

```bash
pip install -r requirements.txt
python dashboard/dashboard.py
```

## Linting and Code Style

**Max line length: 120 characters** (both Python and C++)

### Linters (all three must pass)

| Linter | Language | Config File |
|--------|----------|-------------|
| `ament_flake8` | Python | `.ament_flake8` (ignores E501, W503) |
| `ament_cpplint` | C++ | `.ament_cpplint` (filters: `-build/include_order,-whitespace/line_length`) |
| `ament_uncrustify` | C++/C | `cfg/uncrustify_ros2.cfg` |

**Run all linters:**
```bash
./run_linters.sh [<workspace_dir>]
```

Or individually:
```bash
source /opt/ros/humble/setup.bash
ament_cpplint src
ament_uncrustify src
ament_flake8 src
```

## Testing

### Python tests
```bash
pytest tests/
```

Configuration in `pyproject.toml`:
- Test paths: `tests/`
- Test files: `test_*.py`
- Test classes: `Test*`
- Test functions: `test_*`

### ROS 2 tests (colcon)
```bash
source /opt/ros/humble/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### Dashboard tests
```bash
pytest dashboard/tests/
```

## CI/CD Workflows

All workflows run on **Ubuntu 22.04** with **ROS 2 Humble**.

| Workflow | File | Trigger | Purpose |
|----------|------|---------|---------|
| ROS Humble CI | `ci.yml` | push to main, PRs | Build (linters off) + unit tests |
| ROS 2 CI | `ros2-ci.yml` | push to main, PRs | Build + Debian pkg + tests + linters |
| Debian Release | `debian-release.yml` | tags (`v*`) | Matrix build (amd64/arm64), GitHub release |
| Arduino CI | `arduino-ci.yml` | changes to `.ino` files | Compile Arduino sketches |

The CI uses a custom setup action at `.github/actions/setup-ros/` to install ROS 2.

**CI build flags:**
- Build: `colcon build --cmake-args -DENABLE_LINT_TESTS=OFF`
- Linters run as a separate step after build/test

## Key Conventions

### Code Organization
- **C++** for performance-critical nodes (control, planning, hardware drivers, CAN/ISOBUS)
- **Python** for higher-level nodes (GPS interface, navigation, mission UI, localization)
- Each functional subsystem lives in its own ROS 2 package under `src/`
- Launch files follow a hierarchy: `tractobots_bringup` and `tractobots_launchers` are entry points

### ROS 2 Patterns
- Parameters configured via YAML files
- Launch files in Python (`*_launch.py` or `*.launch.py`)
- Standard message types from `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`
- TF2 for coordinate frame transforms
- WebSocket bridge (rosbridge_websocket on port 9090) connects the Windows dashboard

### Package Naming
- Tractobots packages: `tractobots_<subsystem>`
- Vendor/external packages keep their original names (e.g., `Advancednavigation`, `Gcode_parser`, `agisostack_plus_plus`)

### File Naming
- Python: snake_case (`my_node.py`)
- C++ source: snake_case (`my_node.cpp`, `my_node.hpp`)
- Launch files: `<name>_launch.py` or `<name>.launch.py`

## Architecture Overview

```
Windows Host
  └── Dashboard (PyQt5) ←── WebSocket (port 9090) ──→ rosbridge_websocket
                                                           │
WSL2 / Linux                                               │
  └── ROS 2 Jazzy ─────────────────────────────────────────┘
        ├── tractobots_core (E-stop, diagnostics)
        ├── tractobots_vehicle (CAN/J1939)
        ├── tractobots_planning (Fields2Cover paths)
        ├── tractobots_control (steering, implements)
        ├── tractobots_navigation (driver/teleop)
        ├── tractobots_bridges (AgOpenGPS, NMEA)
        ├── tractobots_gps + Advancednavigation (GPS/INS)
        ├── tractobots_robot_localization (EKF fusion)
        ├── tractobots_nav2 (Navigation2 stack)
        └── tractobots_description (URDF/TF)

Hardware
  ├── Advanced Navigation INS (RTK GPS/IMU)
  ├── Arduino-JD6330 (John Deere implement)
  └── Arduino-MT765 (Massey Ferguson implement)
```

## Dependencies

### Python (from `requirements.txt` and `pyproject.toml`)
- `rclpy`, `sensor-msgs`, `geometry-msgs`, `nav-msgs`, `std-msgs` (ROS 2)
- `tf2-ros`, `tf2-geometry-msgs` (transforms)
- `PyQt5` (dashboard GUI)
- `roslibpy` (WebSocket bridge client)
- `numpy`, `scipy` (computation)
- `matplotlib` (visualization)
- `pyserial` (serial communication)

### System / ROS 2
- ROS 2 Humble/Jazzy
- Navigation2 stack
- Robot Localization package
- Gazebo (simulation)
- can-utils (CAN bus)

## Common Tasks

**Source environment:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_tractobots/install/setup.bash
```

**Launch the full system (Linux):**
```bash
./launch_complete_system.sh
```

**Launch the full system (Windows):**
```powershell
.\launch_complete_system.ps1
```

**One-click setup (Windows):**
```powershell
.\setup_complete_environment.ps1
```

## Type Checking

Pyright is configured via `pyrightconfig.json` for Python type checking (Python 3.10 target).
