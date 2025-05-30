# 🧠 Tractobots Full Autonomy Roadmap

This README outlines a complete plan to evolve Tractobots into a **fully autonomous row-by-row field robot**, capable of executing navigation, steering, tool control, and logging — with minimal operator input.

---

## ✅ Current Capabilities

- GPS + INS fused pose estimation (Advanced Navigation + EKF)
- MapViz visualization with offline satellite maps
- Joystick-based teleoperation with Nav-mode support
- G-code file parser and waypoint path publisher
- ISOBUS CAN monitoring via `iso_bus_watchdog`

---

## 🚀 Phase 1: Navigation Autonomy

### Goal: Drive autonomously using GPS waypoints from G-code

### Tasks:

- [x] **G-code Reader Node**: Parse GPS-based G-code into `nav_msgs/Path`
- [ ] **Coordinate Conversion**: Convert lat/lon → UTM (or local ENU frame)
- [ ] **Nav2 Integration**:
  - Publish waypoints to Nav2's `/goal_pose` or use `FollowPath`
  - Trigger mission start via joystick or UI
- [ ] **End-of-row Logic**:
  - Auto reverse `N` meters
  - Auto turn & align to next row
- [ ] **Path Completion Detection**: Detect final waypoint and auto-stop

---

## ⚙️ Phase 2: Steering via ISOBUS

### Goal: Replace joystick steering with ISOBUS-controlled steering valve.

### Tasks:

- [ ] **Understand tractor's ISOBUS PGN/Messages for steering**:
  - Identify appropriate PGNs (e.g. PGN 65282 – Steering Command)
  - Use AgIsoStack++ to publish steering messages
- [ ] **Create `isobus_steering_node`**:
  - Subscribes to `/cmd_vel` or `/steering_angle`
  - Translates to ISOBUS messages
- [ ] **CAN Safety Watchdog**:
  - Monitor ACK or status messages from steering module
  - Emergency stop if no ACK or sensor feedback within timeout
- [ ] **Integrate with Nav2 or Pure Pursuit**:
  - Use `/cmd_vel` from Nav2 controller → ISOBUS steering

---

## ⚒️ Phase 3: Implement/Tool Control

### Goal: Control implement (e.g., sprayer, seeder) from G-code `Z` commands or automatic logic.

### Tasks:

- [ ] **Tool Control Node**:
  - Listens to `Z` height values from G-code
  - Publishes relay toggle or CAN control messages
- [ ] **Relay / CAN Integration**:
  - Use GPIO or ISOBUS PGNs to toggle implement hardware
- [ ] **Add `/tool_state` topic for UI & logging**

---

## 🧭 Phase 4: Mission Control UI

### Goal: Start/stop autonomous missions via GUI or web interface

### Tasks:

- [x] **Mission Start/Stop** buttons
- [ ] G-code upload + preview
- [ ] Path visualization (from `/gcode_path`)
- [ ] Emergency stop toggle
- [ ] Relay control overrides

---

## 🛰️ Phase 5: Field Logging + Map Export

### Goal: Log every action + location and export for auditing or record-keeping

### Tasks:

- [ ] **Field Logger Node**:
  - Logs pose, tool state, and timestamps
  - Optional: detect anomaly events (stall, stop, CAN error)
- [ ] **CSV/GeoJSON/KML Export**
  - Per-operation logs: spraying, seeding, coverage zones
  - Offline map overlay (compatible with Google Earth)

---

## 🧪 Experimental Add-ons

- [ ] **Headland Turn Planner**: Add full-turn maneuver generation
- [ ] **Obstacle Avoidance**: Add LiDAR or stereo cam with costmap
- [ ] **FarmDSL Interpreter**: Accept higher-level agronomic scripts

---

## 📦 Suggested Node Breakdown

| Node Name               | Function                                     |
|------------------------|----------------------------------------------|
| `gcode_reader_node`    | Parses GPS waypoints from G-code             |
| `follow_gcode_node`    | Converts waypoints → Nav2 missions           |
| `isobus_steering_node` | Sends ISOBUS messages to control steering    |
| `tool_control_node`    | Toggles implement tools from Z commands      |
| `field_logger_node`    | Records geotagged actions and tool state     |
| `mission_ui_node`      | GUI/Web UI to control missions & display logs|

---

## ✅ Milestone Tracking

| Milestone                         | Status |
|----------------------------------|--------|
| Basic G-code navigation          | ✅     |
| Nav2 integration                 | ☐     |
| ISOBUS steering control          | ☐     |
| Tool automation via Z values     | ☐     |
| End-of-row logic + turns         | ☐     |
| Field logging                    | ☐     |
| Full-mission GUI/Web             | ☐     |

---

## 🤝 Community & Contributions

This is an open roadmap. If you’re building autonomous systems or ISOBUS tools for farming, we’d love your feedback or help extending this framework. Fork and PR welcome!

Licensed under the **Autonomous Tractor Software License (ATSL) 1.0**. Built with care by **Crop Crusaders**.
