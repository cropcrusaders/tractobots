#!/bin/bash
# ROS2 Jazzy Installation Script for WSL Ubuntu 24.04
# Run this script inside WSL to install ROS2 Jazzy

echo "🚀 Starting ROS2 Jazzy installation..."

# Update system and install prerequisites
echo "📦 Updating system packages..."
sudo apt update
sudo apt install -y locales curl gnupg lsb-release software-properties-common

# Set up locale
echo "🌍 Setting up locale..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
echo "📋 Adding ROS2 repository..."
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
echo "🤖 Installing ROS2 Jazzy Desktop..."
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Install additional packages for Tractobots
echo "🔧 Installing additional ROS2 packages..."
sudo apt install -y ros-jazzy-gazebo-ros-pkgs ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-rosbridge-server

# Set up environment
echo "⚙️ Setting up ROS2 environment..."
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

# Test installation
echo "🧪 Testing ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 installation successful!"
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "🎉 You can now use the Tractobots dashboard!"
else
    echo "❌ ROS2 installation failed. Please check the error messages above."
    exit 1
fi

echo "📝 To use ROS2 in new terminal sessions, run: source /opt/ros/jazzy/setup.bash"
echo "   Or restart your WSL session."
