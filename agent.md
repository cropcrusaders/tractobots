# Tractobots System Parts (Agent Notes)

## Purpose
This document enumerates the major parts of the Tractobots system and how they fit together, so new contributors can quickly understand the full stack and where to look for each subsystem.

---

## 1. Top-Level System Overview
Tractobots is an autonomous precision agriculture platform that combines:
- A **ROS 2 Jazzy** robotics stack (running on Ubuntu/WSL).
- A **Windows-native dashboard** for monitoring and control.
- A **simulation environment** (Gazebo) for testing field operations.
- A **web interface** and **ROS bridge** for real-time telemetry and control.

---

## 2. Core Robotics Stack (ROS 2)
**Role:** The heart of the autonomous system; handles navigation, control, and simulation.

**Key components:**
- **ROS 2 Jazzy**: Primary robotics framework.
- **Navigation2**: Path planning and motion execution.
- **Robot Localization**: State estimation for GPS/IMU.
- **ISOBUS integration**: Industry-standard implement control over CAN.

**Key locations:**
- `src/` → ROS 2 packages and related code.
- `cfg/` → Configuration files for ROS components.
- `field_data/` → Sample field boundary data for planning.

---

## 3. Windows Dashboard (Control + Monitoring)
**Role:** User-facing control panel for operations, logs, telemetry, and system launch.

**Key components:**
- **Python/PyQt-based Windows GUI** for live monitoring.
- **Launcher scripts** for one-click startup.
- **WSL port forwarding utilities** for ROS bridge connectivity.

**Key locations:**
- `dashboard/` → Main dashboard codebase.
- `dashboard/utils/` → Port forwarding and utility scripts.
- `dashboard/tests/` → Dashboard tests and minimal builds.
- `dashboard/scripts/` → Launcher scripts.

**Entry point:**
- `start_tractobots_dashboard.cmd` (root) → sets up port forwarding, starts the dashboard.

---

## 4. Web-Based Monitoring & ROS Bridge
**Role:** Real-time telemetry and remote control via browser and WebSocket bridge.

**Key components:**
- **rosbridge server** → exposes ROS topics/services over WebSocket.
- **Web UI** → live map, status, and progress monitoring.

**Common endpoints:**
- Dashboard: `http://localhost:8080`
- ROS bridge: `ws://localhost:9090`

---

## 5. Simulation Environment (Gazebo)
**Role:** Provides realistic field simulation for development and testing.

**Key components:**
- **Gazebo simulation** for 3D environment + physics.
- **Sensor simulation** (GPS, IMU, collisions).
- **Field boundaries and obstacles** modeled from shapefiles or CSV.

---

## 6. Agricultural Operations Modules
**Role:** Domain-specific logic for field work.

**Key components:**
- **Field Planner** → generates optimized coverage paths.
- **Plow Controller** → hydraulic/implement control logic.
- **Progress Monitor** → real-time coverage analytics.
- **Boundary Manager** → handles field edges and obstacles.

---

## 7. Scripts & Automation
**Role:** Setup, install, and launch automation for a complete system.

**Key scripts:**
- `setup_complete_environment.ps1` → one-click Windows + WSL setup.
- `launch_complete_system.ps1` / `launch_complete_system.sh` → full system startup.
- `run_dashboard.ps1` / `run_dashboard.bat` → dashboard launch.
- `install_ros2.sh`, `setup_ros_environment.sh` → ROS install/config.

---

## 8. Documentation & Guides
**Role:** Project knowledge base for setup, architecture, and usage.

**Key documents:**
- `README.md` → high-level overview and setup.
- `README_COMPLETE_SYSTEM.md` → full system description and workflows.
- `DASHBOARD_ORGANIZATION_SUMMARY.md` → dashboard structure details.
- `docs/` → additional guides and references.

---

## 9. Data & Assets
**Role:** Example data for simulations and planning.

**Key locations:**
- `field_data/` → CSV/shapefile field boundary samples.
- `demo_john_deere_data.xml` → example telemetry data.

---

## 10. How the Parts Connect
```
Windows Dashboard
   ↕ (WebSocket via ROS bridge)
ROS Bridge (WSL)
   ↕ (ROS2 topics/services)
ROS 2 Stack (Navigation, Field Planning, Control)
   ↕
Gazebo Simulation + Sensors
```

---

## 11. Quick Orientation Checklist
When onboarding or debugging:
1. **Dashboard issues** → check `dashboard/` + port forwarding tools.
2. **ROS stack errors** → check `src/` and `cfg/`.
3. **Simulation problems** → check Gazebo launch files and models.
4. **Field planning** → check field data and planner modules.
5. **Connectivity problems** → verify rosbridge and ports 8080/9090.

---

*This file is meant as a living map of the system. Update it when major parts move or change.*
