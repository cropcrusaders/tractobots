#!/usr/bin/env bash
# ROS2 Jazzy Installation for Ubuntu 24.04 (Noble)
# Jazzy is the official ROS2 distribution for Ubuntu 24.04

set -e

echo "🚜 Installing ROS2 Jazzy on Ubuntu 24.04"
echo "========================================="

# Check Ubuntu version
OS_CODENAME=$(lsb_release -cs)
echo "📋 Detected Ubuntu codename: $OS_CODENAME"

if [ "$OS_CODENAME" != "noble" ]; then
    echo "⚠️  Warning: This script is designed for Ubuntu 24.04 (Noble)"
    echo "   Your system is: $OS_CODENAME"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo "📦 Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install basic dependencies
echo "🔧 Installing basic dependencies..."
sudo apt install -y \
  curl \
  gnupg2 \
  lsb-release \
  software-properties-common \
  build-essential \
  git \
  python3-pip \
  python3-dev \
  python3-setuptools \
  python3-vcstool

# Enable universe repository
sudo add-apt-repository -y universe
sudo apt update

# Add ROS2 repository
echo "🗝️ Adding ROS2 repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# For Ubuntu 24.04, use the official Noble packages
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu noble main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Install ROS2 Jazzy
echo "🤖 Installing ROS2 Jazzy..."
sudo apt install -y \
  ros-jazzy-desktop \
  python3-rosdep \
  python3-colcon-common-extensions

# Install additional useful packages
echo "📦 Installing additional ROS2 packages..."
sudo apt install -y \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt \
  ros-jazzy-geographic-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-tf2 \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-cartographer \
  ros-jazzy-cartographer-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro

# Initialize rosdep
echo "🔄 Initializing rosdep..."
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Set up environment
echo "🌍 Setting up environment..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Source for this session
source /opt/ros/jazzy/setup.bash

# Verify installation
echo "✅ Verifying ROS2 installation..."
if ros2 --version; then
    echo "🎉 ROS2 Jazzy installed successfully!"
    echo "🔧 ROS_DISTRO: $ROS_DISTRO"
    echo ""
    echo "🚀 Next steps:"
    echo "   1. Close and reopen your terminal"
    echo "   2. Run: source /opt/ros/jazzy/setup.bash"
    echo "   3. Test: ros2 --version"
    echo "   4. Build tractobots: ./quick_setup.sh"
else
    echo "❌ ROS2 installation verification failed"
    exit 1
fi

echo ""
echo "📝 Note: Jazzy is the recommended ROS2 distribution for Ubuntu 24.04"
echo "   If you need Humble compatibility, consider using Docker or Ubuntu 22.04"
