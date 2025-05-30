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
 - **Mission Control UI**: Web page to start/stop missions and trigger an emergency stop

---

## ⚙️ Prerequisites

### 1. ROS 2 Humble on Ubuntu 22.04

```bash
sudo apt update && sudo apt install -y   curl gnupg2 lsb-release software-properties-common

# Add ROS 2 repos & keys
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key   -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]   http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"   | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop   python3-colcon-common-extensions python3-rosdep2 python3-pip
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

---

## 🚀 Usage

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

---

## 📈 Continuous Integration (GitHub Actions)

A CI workflow (`.github/workflows/ros2-ci.yml`) now also builds & lint‑tests:

- `iso_bus_watchdog` alongside the other packages
- SocketCAN integration checks on Ubuntu 22.04 & Humble
- Uses `ros-tooling/setup-ros@v0.7.12` so `apt-get update` works with the current ROS key

Example step:

```yaml
- name: Setup ROS 2
  uses: ros-tooling/setup-ros@v0.7.12
  with:
    required-ros-distributions: humble
```

---

## 📝 License

This project is released under the **Autonomous Tractor Software License (ATSL) 1.0**. See [LICENSE](LICENSE) for details.

`iso_bus_watchdog` is an exception and continues to be distributed under the MIT
license in order to remain compatible with the upstream AgIsoStack++ project.
