# Tractobots Usage Guide

This short guide summarizes the typical steps to build and run the Tractobots stack. It is intended as a quick reference for new users.

## 1. Install Dependencies

Tractobots requires **Ubuntu 22.04** with ROS 2 Humble. The repository includes a helper script that installs all required packages:

```bash
./install_ros2_humble.sh
```

If apt cannot find packages like `libunwind-dev`, enable the `universe`
repository first:

```bash
sudo add-apt-repository -y universe
sudo apt update
```

After installation, remember to source the ROS environment:

```bash
source /opt/ros/humble/setup.bash
```

## 2. Clone and Build

Create a workspace, clone the repository, and build with `colcon`:

```bash
mkdir -p ~/ros2_tractobots/src
cd ~/ros2_tractobots/src
git clone <this repository>
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 3. Launch the Stack

Most launch files live in `src/tractobots_launchers/launch`. To start the sensors and state estimation stack run:

```bash
ros2 launch tractobots_launchers bringup.launch.py
```

In separate terminals start the joystick tele‑op driver:

```bash
ros2 run joy joy_node
ros2 run tractobots_navigation driver
```

Optional launch files for MapViz visualization and pose transforms are also available:

```bash
ros2 launch tractobots_launchers mapviz.launch.py
ros2 launch tractobots_launchers pose_tf.launch.py
```

## 4. Additional Nodes

Other packages provide ISOBUS monitoring, mission control UIs, and navigation helpers. Refer to the main [README](../README.md) for detailed usage instructions.

## 5. Example Paddock Workflow

Once you arrive at the paddock with the tractor powered on, open a few
terminals and source your workspace in each one:

```bash
source ~/ros2_tractobots/install/setup.bash
```

Start the sensors and state estimation stack:

```bash
ros2 launch tractobots_launchers bringup.launch.py
```

In another terminal run the joystick and driver nodes:

```bash
ros2 run joy joy_node
ros2 run tractobots_navigation driver
```

Drive to the start of your first row and hold **Nav + Y** to record the
A–B line. Keep **Nav** pressed and hit **Start** to engage row-by-row
guidance. You can also launch MapViz or the mission control UI for
visual feedback or mission management:

```bash
ros2 launch tractobots_launchers mapviz.launch.py
ros2 run tractobots_mission_ui mission_gui_node
```

Stop the nodes with **Ctrl+C** when finished.

