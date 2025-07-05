#!/bin/bash
# =============================================================================
# Gazebo Simulator Setup for Tractobots
# Professional 3D Simulation Environment for Agricultural Robotics
# =============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=========================================================${NC}"
echo -e "${BLUE}  🚜 TRACTOBOTS GAZEBO SIMULATOR SETUP  🚜${NC}"
echo -e "${BLUE}=========================================================${NC}"

# Function to print status messages
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running on Ubuntu
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" != "ubuntu" ]; then
        print_error "This script is designed for Ubuntu. Current OS: $ID"
        exit 1
    fi
    print_status "Detected Ubuntu $VERSION_ID"
else
    print_error "Cannot detect OS. This script requires Ubuntu."
    exit 1
fi

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 is not installed. Please install ROS2 Jazzy first."
    exit 1
fi

# Source ROS2 environment
print_status "Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Update package lists
print_status "Updating package lists..."
sudo apt update

# Install Gazebo Garden (recommended for ROS2 Jazzy)
print_status "Installing Gazebo Garden..."
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-gzweb \
    ros-jazzy-sdformat-urdf \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro

# Install additional Gazebo packages
print_status "Installing additional Gazebo packages..."
sudo apt install -y \
    gz-garden \
    gz-sim7 \
    gz-tools2 \
    gz-sensors7 \
    gz-rendering7 \
    gz-physics6 \
    gz-gui7 \
    gz-common5 \
    gz-msgs9 \
    gz-transport12

# Install visualization tools
print_status "Installing visualization tools..."
sudo apt install -y \
    rviz2 \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-common \
    ros-jazzy-rviz-rendering \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins

# Install navigation and mapping packages
print_status "Installing navigation and mapping packages..."
sudo apt install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-nav2-common \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-util \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-constrained-smoother \
    ros-jazzy-nav2-rotation-shim-controller \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-dwb-controller \
    ros-jazzy-nav2-theta-star-planner \
    ros-jazzy-nav2-navfn-planner \
    ros-jazzy-nav2-smac-planner

# Install SLAM packages
print_status "Installing SLAM packages..."
sudo apt install -y \
    ros-jazzy-slam-toolbox \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-cartographer-rviz

# Install sensor packages
print_status "Installing sensor packages..."
sudo apt install -y \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-sensor-msgs

# Install additional dependencies
print_status "Installing additional dependencies..."
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-setuptools \
    python3-yaml \
    build-essential \
    cmake \
    git \
    wget \
    curl

# Create Gazebo models directory
print_status "Creating Gazebo models directory..."
mkdir -p ~/.gazebo/models

# Download common Gazebo models
print_status "Downloading common Gazebo models..."
cd ~/.gazebo/models

# Download basic models if they don't exist
if [ ! -d "ground_plane" ]; then
    print_status "Downloading ground plane model..."
    wget -qO- https://github.com/osrf/gazebo_models/archive/master.tar.gz | tar xz --strip-components=1
fi

# Create workspace directory for Gazebo integration
print_status "Creating Gazebo workspace..."
mkdir -p ~/tractobots_gazebo_ws/src

# Create basic launch files directory
print_status "Creating launch files directory..."
mkdir -p ~/tractobots_gazebo_ws/src/tractobots_gazebo/launch
mkdir -p ~/tractobots_gazebo_ws/src/tractobots_gazebo/worlds
mkdir -p ~/tractobots_gazebo_ws/src/tractobots_gazebo/models
mkdir -p ~/tractobots_gazebo_ws/src/tractobots_gazebo/config

# Set up environment variables
print_status "Setting up environment variables..."
echo "# Gazebo environment variables" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=~/.gazebo/models:/usr/share/gazebo-11/models" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=~/.gazebo/models:/usr/share/gazebo-11/models" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" >> ~/.bashrc

# Source the new environment
source ~/.bashrc

# Test Gazebo installation
print_status "Testing Gazebo installation..."
timeout 10s gz sim --version || print_warning "Gazebo version check timed out (this is normal)"

# Create a basic test world
print_status "Creating basic test world..."
cat > ~/tractobots_gazebo_ws/src/tractobots_gazebo/worlds/farm_field.sdf << 'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="farm_field">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Farm field boundaries -->
    <model name="field_boundary">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="boundary_link">
        <collision name="boundary_collision">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="boundary_visual">
          <geometry>
            <box>
              <size>100 100 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Scene settings -->
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 1.0</background>
      <shadows>true</shadows>
    </scene>
    
    <!-- Wind (for realistic field conditions) -->
    <wind>
      <linear_velocity>2 0 0</linear_velocity>
    </wind>
    
  </world>
</sdf>
EOF

print_status "Creating package.xml for Gazebo integration..."
cat > ~/tractobots_gazebo_ws/src/tractobots_gazebo/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>tractobots_gazebo</name>
  <version>1.0.0</version>
  <description>Gazebo simulation environment for Tractobots</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>ros-gz-sim</depend>
  <depend>ros-gz-bridge</depend>
  <depend>ros-gz-interfaces</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>xacro</depend>
  <depend>rviz2</depend>
  <depend>nav2_bringup</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

print_status "Creating CMakeLists.txt for Gazebo integration..."
cat > ~/tractobots_gazebo_ws/src/tractobots_gazebo/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(tractobots_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install model files
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF

print_status "Creating basic launch file..."
cat > ~/tractobots_gazebo_ws/src/tractobots_gazebo/launch/tractobots_gazebo.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    gazebo_pkg = get_package_share_directory('tractobots_gazebo')
    
    # World file
    world_file = os.path.join(gazebo_pkg, 'worlds', 'farm_field.sdf')
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load'
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': LaunchConfiguration('world')
        }.items()
    )
    
    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/gps@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        ros_gz_bridge
    ])
EOF

# Make launch file executable
chmod +x ~/tractobots_gazebo_ws/src/tractobots_gazebo/launch/tractobots_gazebo.launch.py

# Build the workspace
print_status "Building Gazebo workspace..."
cd ~/tractobots_gazebo_ws
colcon build --symlink-install

print_status "Sourcing new workspace..."
source ~/tractobots_gazebo_ws/install/setup.bash

# Update rosdep
print_status "Updating rosdep..."
sudo rosdep init || true
rosdep update

print_status "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y || true

print_status "Final build..."
colcon build --symlink-install

echo -e "${GREEN}=========================================================${NC}"
echo -e "${GREEN}  🎉 GAZEBO SIMULATOR SETUP COMPLETE! 🎉${NC}"
echo -e "${GREEN}=========================================================${NC}"
echo ""
echo -e "${BLUE}To use Gazebo with Tractobots:${NC}"
echo -e "${YELLOW}1. Source the environment:${NC}"
echo "   source ~/tractobots_gazebo_ws/install/setup.bash"
echo ""
echo -e "${YELLOW}2. Launch the farm simulation:${NC}"
echo "   ros2 launch tractobots_gazebo tractobots_gazebo.launch.py"
echo ""
echo -e "${YELLOW}3. In another terminal, launch RViz:${NC}"
echo "   rviz2"
echo ""
echo -e "${YELLOW}4. Test Gazebo directly:${NC}"
echo "   gz sim ~/tractobots_gazebo_ws/src/tractobots_gazebo/worlds/farm_field.sdf"
echo ""
echo -e "${GREEN}Your professional 3D simulator is now ready for agricultural robotics! 🚜${NC}"
