# Gazebo Installation Guide for ROS2 Jazzy on Ubuntu 24.04

## Issue
The standard Gazebo packages for ROS2 Jazzy on Ubuntu 24.04 are not available in the default repositories.

## Solutions

### Option 1: Install Gazebo Harmonic (Recommended)
```bash
# Install Gazebo Harmonic (latest stable)
sudo apt-get update
sudo apt-get install lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install gz-harmonic

# Install ROS2-Gazebo bridge (if available)
sudo apt-get install ros-jazzy-ros-gz
```

### Option 2: Build from Source
```bash
# Install dependencies
sudo apt-get install git cmake build-essential

# Clone and build ros_gz packages
mkdir -p ~/gz_ws/src
cd ~/gz_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b ros2

cd ~/gz_ws
colcon build --packages-select ros_gz_sim ros_gz_bridge
source install/setup.bash
```

### Option 3: Use Docker
```bash
# Pull ROS2 Jazzy with Gazebo
docker pull osrf/ros:jazzy-desktop-full

# Run with display forwarding
docker run -it --rm \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  osrf/ros:jazzy-desktop-full
```

### Option 4: Use Alternative Simulator
Consider using:
- **Isaac Sim** (NVIDIA's robotics simulator)
- **Webots** (Open-source robot simulator)
- **CoppeliaSim** (formerly V-REP)

## Testing Installation
After installation, test with:
```bash
# For Gazebo Harmonic
gz sim

# For ROS2 bridge
ros2 pkg list | grep gz
```

## Next Steps
1. Choose one of the above options
2. Update the launch files accordingly
3. Test the tractor model spawning
4. Integrate with the dashboard

## Dashboard Integration
Once Gazebo is installed, the dashboard "Launch Gazebo Simulation" button will work with the `gazebo_tractor.launch.py` file.
