# Codex Guide for Tractobots

This short guide explains how Codex or any automated tooling can set up and run
the Tractobots code base for analysis or testing purposes.

## 1. System Dependencies

Tractobots targets **Ubuntu 22.04** with ROS 2 Humble. The easiest way to set
up all required packages is to run the helper script in the repository root:

```bash
./install_ros2_humble.sh
```

It installs ROS 2, Python build tools, and initializes `rosdep`. Make sure to
`source /opt/ros/humble/setup.bash` after installation.

If you plan to run the package tests or linters, also install the ROS 2
`ament_*` helpers:

```bash
sudo apt install ros-humble-ament-cmake-test \
  ros-humble-ament-cmake-gtest ros-humble-ament-cmake-pytest \
  ros-humble-ament-lint-auto ros-humble-ament-lint-common
```

## 2. Building the Workspace

Clone the repository inside a ROS 2 workspace and build with `colcon`:

```bash
mkdir -p ~/ros2_tractobots/src
cd ~/ros2_tractobots/src
git clone <this repo>
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 3. Launching the Stack

Common launch files live in `src/tractobots_launchers/launch`. To bring up the
sensors and state estimation stack, run:

```bash
ros2 launch tractobots_launchers bringup.launch.py
```

Additional launch files exist for MapViz and pose transforms.

## 4. Running Tests

Some packages include unit tests using `pytest` or `ament_cmake`. After
building the workspace, ensure the ROS environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then run the tests with:

```bash
colcon test
colcon test-result --all
```

## 5. Where to Look

- Core launch files: `src/tractobots_launchers/launch`
- Navigation driver: `src/tractobots_navigation/tractobots_navigation/driver.py`
- G-code parser: `src/Gcode_parser`
- Robot description: `src/tractobots_description/urdf`

These locations provide a starting point when inspecting how the code works.

