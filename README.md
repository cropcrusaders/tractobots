# 🚜 Tractobots Project (Reincarnated by Crop Crusaders)

Field robots for precision agriculture — now super‑powered on **ROS 2 (Humble Hawksbill)**!

---

## 🔥 What’s New & Exciting

- **ROS 2-native**: Fully ported to ROS 2 with `colcon` builds & Python‑launch scripts  
- **High‑precision INS support**: Plug in an Advanced Navigation INS for RTK‑level, fused GPS + IMU data  
- **Modular “bringup”**: One‑line launch of your entire stack—sensors, state estimation, teleop & autonomy  
- **MapViz integration**: Offline Google Maps + live pose/TF visualization  
- **Row-by-Row Guidance**: Precise A–B line following for autonomous pass‑by‑pass tractor operation  
- **ISOBUS Watchdog Node**: New C++ node using AgIsoStack++ to monitor engine oil pressure, fuel pressure, coolant temp, and fuel level with auto cut‑out and emergency‑stop  
 - **Mission Control UI**: Web page or Tkinter GUI to start/stop missions and trigger an emergency stop

---

## ⚙️ Prerequisites

### 🪟 Windows + WSL Setup (Optional)

If you are developing on **Windows 10/11**, the easiest way to run the Linux
ROS 2 stack is through **WSL 2**. In an elevated PowerShell run:

```powershell
wsl --install
```

Reboot if prompted and make sure version 2 is the default:

```powershell
wsl --set-default-version 2
```

Install the Ubuntu 22.04 distribution if it was not installed automatically:

```powershell
wsl --install -d Ubuntu-22.04
```

Start the new Ubuntu terminal from the Start menu and follow all remaining
Linux commands in this README from inside that shell. For GUI tools such as
MapViz or RViz you also need an X server on Windows (e.g. **VcXsrv**) and must
export the display inside WSL:

```bash
export DISPLAY=:0
```

### 1. ROS 2 Humble on Ubuntu 22.04

> **Note**
> Tractobots currently targets **Ubuntu 22.04 (Jammy)**. Running the
> installation steps on Ubuntu 24.04 (Noble) may cause `rosdep` to fail with
> missing packages such as `libunwind-dev`. Ensure the **universe** repository is
> enabled (`sudo add-apt-repository -y universe`) if apt cannot locate these
> packages. For best results use a Jammy-based system or build ROS 2 and Nav2
> from source on your distribution.

```bash
sudo apt update && sudo apt install -y   curl gnupg2 lsb-release software-properties-common

# Enable the universe repo for packages like libunwind-dev
sudo add-apt-repository -y universe
sudo apt update

# Add ROS 2 repos & keys
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]   http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"   | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop   python3-colcon-common-extensions \
  python3-rosdep2 python3-empy python3-pip \
  libunwind-dev libgoogle-glog-dev

# Remove any conflicting 'em' package if present
pip3 uninstall -y em 2>/dev/null || true
```

You can also run the provided helper script for a non-interactive setup:

```bash
./install_ros2_humble.sh
```

Initialize **rosdep** (first time only):

```bash
sudo rosdep init
rosdep update
```

### 2. SocketCAN & CAN‑utils

We use SocketCAN to interface with your ISOBUS/CAN hardware:

```bash
sudo apt install -y can-utils
# Bring up your CAN interface (adjust 'can0' and bitrate as needed)
sudo ip link set can0 up type can bitrate 250000
```

### 3. Docker (Optional)

Run Daniel Snider’s MapProxy for offline Google/Satellite tiles:

```bash
mkdir -p ~/mapproxy
docker run -d -p 8080:8080 -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

MapViz can now point at `http://localhost:8080/services/tile.xml`.

## 🛠️ Hardware Setup

Below is a reference setup for running Tractobots on real machinery.  The
software can adapt to different tractors or computers, but the following
components are known to work well:

1. **Computer** – Ubuntu 22.04 PC or Jetson class SBC with at least 4 GB of RAM.
   Connect all sensors and the CAN adapter to this machine.
2. **GPS / INS** – An Advanced Navigation INS (e.g. Spatial or M2) provides
   RTK‑level pose data via serial or USB.  Mount the antenna on the tractor roof
   and connect the unit to the computer.
3. **CAN Interface** – A SocketCAN compatible adapter (PEAK PCAN‑USB,
   Innomaker usb2can, MCP2515 hat, etc.) connects the onboard computer to the
   tractor’s ISOBUS/CAN wiring. Bring the device up as `can0` at
   `250000 bit/s` as shown below.
4. **Joystick** – Any USB gamepad recognized by `joy_node` (Xbox or PS4) can be
   used for manual control and mission start/stop commands.
5. **Optional Sensors** – LiDAR, cameras or implement controllers can be added
   for advanced autonomy. Ensure power and cabling are secured before operating
   in the field.

With everything wired up and powered on you can follow the bringup steps in the
next section to launch the stack.

---

## 📥 Clone & Build

1. **Create your ROS 2 workspace and clone this repo**

    ```bash
    mkdir -p ~/ros2_tractobots/src
    cd ~/ros2_tractobots/src
    git clone https://github.com/cropcrusaders/tractobots.git
    cd ~/ros2_tractobots
    ```

    This single repository already contains all Tractobots packages and
    drivers, including the Advanced Navigation INS driver and AgIsoStack++.

2. **Install dependencies**

    ```bash
    # System packages for INS driver + numpy + nvector
    sudo apt install -y python3-serial python3-numpy python3-nvector
    pip3 install pynmea2

    # SocketCAN tools
    sudo apt install -y can-utils

    # ROS package dependencies
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **Build & source**

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

    Or simply run the convenience script from this repository:

    ```bash
    ./build_workspace.sh
    ```

---

## 🛠️ Detailed Build & Run

Below is a full walkthrough to compile the workspace and run either a simulated
environment or the real tractor.

### 1. Compile the workspace

```bash
cd ~/ros2_tractobots
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Alternatively run the helper script from the repository root:

```bash
./build_workspace.sh
```

### 2. Running a simulation

You can test the stack without any physical hardware attached. Launch the
sensors and state estimation, then open MapViz to visualize the robot:

```bash
ros2 launch tractobots_launchers bringup.launch.py
ros2 launch tractobots_launchers mapviz.launch.py  # optional GUI
```

Use the joystick driver to send manual commands while observing odometry in
MapViz:

```bash
ros2 run joy joy_node
ros2 run tractobots_navigation driver
```

### 3. Running on real hardware

Connect your Advanced Navigation INS and bring up the CAN interface:

```bash
sudo ip link set can0 up type can bitrate 250000
```

Start the standard bringup along with teleoperation or your autonomous mission:

```bash
ros2 launch tractobots_launchers bringup.launch.py
ros2 run joy joy_node
ros2 run tractobots_navigation driver
```

For safety-critical monitoring you can add the ISOBUS watchdog node:

```bash
ros2 run iso_bus_watchdog iso_bus_watchdog_node
```

---

## 🚀 Usage

If you want a short reference with the basic build and run steps,
see [docs/USAGE_GUIDE.md](docs/USAGE_GUIDE.md).
When you roll into the paddock, follow the **Example Paddock Workflow**
section in that guide for a quick start.

Open **five** terminals, each sourcing your workspace:

```bash
source ~/ros2_tractobots/install/setup.bash
```

### 1. Bringup: Sensors → State Estimation → INS

```bash
ros2 launch tractobots_launchers bringup.launch.py
```

Starts:

- `robot_state_publisher` (URDF TF tree)  
- `gps_parser`, `imu_publisher` (legacy)  
- `advanced_navigation_driver` (INS)  
- `navsat_transform_node` & `ekf_node` → fused `/odometry/filtered`

### 2. Tele‑op & Driver

```bash
ros2 run joy joy_node
ros2 run tractobots_navigation driver
```

Joystick controls:

- **Enable/Disable** engine (Start/Back buttons)  
- **Steer** with left stick, **Throttle** with triggers  
- **Hitch Up/Down** with right stick Y  
- **Nav‑mode** (hold Nav + A/B/X/Y) for line‑following

### 3. MapViz Visualization

```bash
ros2 launch tractobots_launchers mapviz.launch.py
```

- Fixed Frame: `map` or `odom`  
- Displays: `/odometry/filtered`, `/gps/marker`, offline map tiles

### 4. Optional Pose Transformer

```bash
ros2 launch tractobots_launchers pose_tf.launch.py
```

Broadcasts `base_link → utm` TF for georeferenced transforms.

### 5. ISOBUS Watchdog Node

Monitor engine vitals and trigger emergency‑stop:

```bash
# bring up CAN interface if not already done
sudo ip link set can0 up type can bitrate 250000

# run the node (with optional threshold overrides)
ros2 run iso_bus_watchdog iso_bus_watchdog_node   --ros-args     -p oil_pressure_min:=150.0     -p oil_pressure_max:=700.0     -p coolant_temp_min:=0.0     -p coolant_temp_max:=90.0     -p fuel_level_min:=10.0     -p fuel_level_max:=90.0
```

Published topics:

- `/engine/oil_pressure` (kPa)  
- `/engine/fuel_pressure` (kPa)  
- `/engine/coolant_temp` (°C)  
- `/engine/fuel_level` ( %)  
- `/emergency_stop` (Bool)

### 6. Row‑by‑Row (Pass‑by‑Pass) Guidance

1. **Record A–B line**: In your driver, hold Nav + Y to start “north” line at current GPS.  
2. **Follow**: The Line Follower algorithm computes cross‑track error and steers accordingly.  
3. **Repeat**: At end‑of‑row, manually switch row or extend logic for auto headland turns.

### 7. Mission Control UI

A small Tkinter window can start or stop missions and toggle the emergency stop.

```bash
ros2 run tractobots_mission_ui mission_gui_node
```

The original web interface is still available on port 8088 when running
`mission_ui_node`.

### 8. Nav2 Autonomous Navigation

A minimal Nav2 launch file is now included. It starts the standard
`nav2_bringup` stack with parameters tailored for Tractobots:

```bash
ros2 launch tractobots_nav2 nav2.launch.py
```

The stack subscribes to `/goal_pose` or `/navigate_to_pose` actions and
publishes `/cmd_vel` for the driver or steering controller.

---

## 📈 Continuous Integration (GitHub Actions)

A CI workflow (`.github/workflows/ros2-ci.yml`) now also builds & lint‑tests:

- `iso_bus_watchdog` alongside the other packages
- SocketCAN integration checks on Ubuntu 22.04 & Humble
- CI runner explicitly uses `ubuntu-22.04` so ROS 2 packages install correctly
- Uses the vendored `.github/actions/setup-ros` action so `apt-get update` works with the current ROS key
- The action sets `BASH_ENV` so the ROS environment is sourced for all subsequent steps

Example step:

```yaml
- name: Setup ROS 2
  uses: ./.github/actions/setup-ros
  # No additional inputs required
```

## 🤖 Codex Quickstart

For a short overview of how to build and test Tractobots in automated setups,
see [docs/CODEX_GUIDE.md](docs/CODEX_GUIDE.md). It lists the basic steps to
install ROS 2, build the workspace with `colcon`, and run unit tests.

If you are looking for the source location of each ROS 2 node in this
repository, check the table in
[docs/NODE_REFERENCES.md](docs/NODE_REFERENCES.md).

Instructions for creating simple waypoint files in G-code format are available
in [docs/GCODE_GENERATOR.md](docs/GCODE_GENERATOR.md).

---

## 📝 License

This project is released under the **Autonomous Tractor Software License (ATSL) 1.0**. See [LICENSE](LICENSE) for details.

`iso_bus_watchdog` is an exception and continues to be distributed under the MIT
license in order to remain compatible with the upstream AgIsoStack++ project.

<!-- Dummy update to trigger CI -->
