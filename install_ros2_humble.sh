#!/usr/bin/env bash
set -e

# ROS 2 Humble is officially supported on Ubuntu 22.04 (Jammy)
OS_CODENAME=$(lsb_release -cs)
if [[ "$OS_CODENAME" != "jammy" ]]; then
  echo "ROS 2 Humble packages are only available for Ubuntu 22.04 (Jammy)." >&2
  echo "Detected codename: $OS_CODENAME" >&2
  echo "Consider using a Jammy-based system or building ROS 2 from source on this OS." >&2
  exit 1
fi

sudo apt update && sudo apt install -y \
  curl gnupg2 lsb-release software-properties-common

# Enable the 'universe' repository for packages like libunwind-dev
sudo add-apt-repository -y universe
sudo apt update

# Add the ROS 2 repository and key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $OS_CODENAME main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep2 \
  python3-empy \
  python3-pip \
  libunwind-dev \
  libgoogle-glog-dev

# Remove conflicting 'em' package if present
pip3 uninstall -y em 2>/dev/null || true

# Initialize rosdep if needed
if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep not found; ensure python3-rosdep2 installed correctly" >&2
fi
sudo rosdep init || true
rosdep update

