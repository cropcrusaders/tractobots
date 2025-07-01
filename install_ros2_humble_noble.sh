#!/usr/bin/env bash
# ROS2 Humble Installation for Ubuntu 24.04 (Noble)
# Fixed version that works with newer Ubuntu

set -e

echo "🚜 Installing ROS2 Humble on Ubuntu 24.04"
echo "=========================================="

# Check Ubuntu version
OS_CODENAME=$(lsb_release -cs)
echo "📋 Detected Ubuntu codename: $OS_CODENAME"

# Update system
echo "📦 Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install dependencies
echo "🔧 Installing dependencies..."
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
  python3-vcstool \
  python3-rosdep \
  python3-colcon-common-extensions

# Enable universe repository
sudo add-apt-repository -y universe
sudo apt update

# Add ROS2 repository
echo "🗝️ Adding ROS2 repository..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# For Ubuntu 24.04, use jammy packages (they work fine)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Install ROS2 Humble
echo "🤖 Installing ROS2 Humble..."
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-rmw-cyclonedx-cpp \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-geographic-msgs \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs

# Install additional useful packages
echo "📦 Installing additional ROS2 packages..."
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro

# Initialize rosdep
echo "🔄 Initializing rosdep..."
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Set up environment
echo "🌍 Setting up environment..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Source for this session
source /opt/ros/humble/setup.bash

# Verify installation
echo "✅ Verifying ROS2 installation..."
if ros2 --version; then
    echo "🎉 ROS2 Humble installed successfully!"
    echo "🔧 ROS_DISTRO: $ROS_DISTRO"
    echo ""
    echo "🚀 Next steps:"
    echo "   1. Close and reopen your terminal"
    echo "   2. Run: source /opt/ros/humble/setup.bash"
    echo "   3. Test: ros2 --version"
    echo "   4. Build tractobots: ./quick_setup.sh"
else
    echo "❌ ROS2 installation verification failed"
    exit 1
fi
