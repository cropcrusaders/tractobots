# 🚜 Tractobots Project (Reincarnated by Crop Crusaders)

Field robots for precision agriculture — now super‑powered on **ROS 2 (Humble Hawksbill)**!

---

## 🔥 What’s New & Exciting

- **ROS 2-native**: Fully ported to ROS 2 with `colcon` builds & Python‑launch scripts  
- **High‑precision INS support**: Plug in an Advanced Navigation INS for RTK‑level, fused GPS + IMU data  
- **Modular “bringup”**: One-line launch of your entire stack—sensors, state estimation, teleop & autonomy  
- **MapViz integration**: Offline Google Maps + live pose/TF visualization  
- **Row-by-Row Guidance**: Precise A–B line following for autonomous pass‑by‑pass tractor operation  

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

Initialize **rosdep** (first time only):

```bash
sudo rosdep init
rosdep update
```

---

### 2. Docker (Optional)

Run Daniel Snider’s MapProxy for offline Google/Satellite tiles:

```bash
mkdir -p ~/mapproxy
docker run -d -p 8080:8080 -v ~/mapproxy:/mapproxy danielsnider/mapproxy
```

MapViz can now point at `http://localhost:8080/services/tile.xml`.

---

## 📥 Clone & Build

1. **Create your ROS 2 workspace**

    ```bash
    mkdir -p ~/ros2_tractobots/src
    cd ~/ros2_tractobots
    ```

2. **Clone your Tractobots packages + INS driver**

    ```bash
    cd src
    git clone https://github.com/kylerlaird/tractobots_description.git
    git clone https://github.com/kylerlaird/tractobots_gps.git
    git clone https://github.com/kylerlaird/tractobots_robot_localization.git
    git clone https://github.com/kylerlaird/tractobots_navigation.git
    git clone https://github.com/kylerlaird/tractobots_launchers.git
    git clone https://github.com/advanced-navigation/ros2-driver.git
    cd ~/ros2_tractobots
    ```

3. **Install all dependencies**

    ```bash
    # System packages for INS driver + numpy + nvector
    sudo apt install -y python3-serial python3-numpy python3-nvector
    pip3 install pynmea2

    # ROS package dependencies
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. **Build & source**

    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## 🚀 Usage

Open **four** terminals, each sourcing your workspace:

```bash
source ~/ros2_tractobots/install/setup.bash
```

### 1. Bringup: Sensors → State Estimation → INS

```bash
ros2 launch tractobots_launchers bringup.launch.py
```

This starts:

- **robot_state_publisher** (URDF TF tree)  
- **gps_parser**, **imu_publisher** (legacy)  
- **advanced_navigation_driver** (INS)  
- **navsat_transform_node** & **ekf_node** → fused `/odometry/filtered`

### 2. Tele‑op & Driver

```bash
ros2 run joy joy_node
ros2 run tractobots_navigation driver
```

Use your joystick to:

- **Enable/Disable** engine (Start/Back buttons)  
- **Steer** with left stick, **Throttle** with triggers  
- **Hitch Up/Down** with right stick Y  
- **Nav‑mode** (hold Nav button + A/B/X/Y) for line‑following

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

---

## 🌾 Row‑by‑Row (Pass‑by‑Pass) Guidance

1. **Record A–B line**: In your driver, hold Nav + Y to start “north” line at current GPS.  
2. **Follow**: The **Line Follower** algorithm computes cross‑track error and steers accordingly.  
3. **Repeat**: At end‑of‑row (detected by distance or geofence), manually switch row or extend logic for autonomous headland turn.

For advanced AB‑line management and Pure Pursuit steering, consider adding a **Pure Pursuit** ROS 2 node that subscribes to `/odometry/filtered` and your waypoint list, outputting steering angles to `/navigation/steering_pid/command`.

---

## 📈 Continuous Integration (GitHub Actions)

A CI workflow (`.github/workflows/ros2-ci.yml`) is included to:

- **Checkout** submodules (driver & Tractobots code)  
- **Install** all apt & pip deps  
- **Build & test** every package on Ubuntu latest  

Your PRs will automatically verify build health across ROS 2 Kinetic → Humble transitions!

---

## 📝 License

**GPL v3**. See [LICENSE](LICENSE).
