# 🚜 Tractobots - Getting Started Guide

## Quick Setup for Working System

### Prerequisites
- Windows 10/11 with WSL2 
- Ubuntu 22.04 in WSL
- 8GB+ RAM recommended
- USB ports for hardware connections

### Option A: Automated Windows Setup (Recommended)
```powershell
# Run in elevated PowerShell (right-click -> Run as Administrator)
cd "C:\Users\nicholas\OneDrive\Documents\GitHub\tractobots"
.\setup_windows.ps1
```

### Option B: Manual Setup

#### Step 1: WSL2 Setup (Windows Users)
```powershell
# Run in elevated PowerShell
wsl --install -d Ubuntu-22.04
wsl --set-default-version 2
```

#### Step 2: Install ROS2 Humble (in WSL Ubuntu)
```bash
cd /mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots
chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
source /opt/ros/humble/setup.bash
```

#### Step 3: Quick Build & Test
```bash
chmod +x quick_setup.sh
./quick_setup.sh
```

### Testing Your Setup

#### Basic System Test
```bash
source ~/ros2_tractobots/install/setup.bash
ros2 run tractobots_bringup system_test
```

#### Full System Test
```bash
# Terminal 1 - Start basic sensors
ros2 launch tractobots_launchers bringup.launch.py

# Terminal 2 - Test joystick input (optional)
ros2 run joy joy_node

# Terminal 3 - Start driver
ros2 run tractobots_navigation driver
```

#### Visualization (Optional)
```bash
# Install X server on Windows (VcXsrv) first
export DISPLAY=:0
ros2 launch tractobots_launchers mapviz.launch.py
```

## Hardware Connections

### Basic Setup (Simulation/Testing)
- USB Joystick/Gamepad
- GPS device (optional for testing)

### Full Autonomous Setup
1. **GPS/INS**: Advanced Navigation device via Serial/USB
2. **Tractor Control**: Arduino Uno/Mega connected to tractor systems
3. **CAN Interface**: USB-CAN adapter for ISOBUS communication
4. **Safety Systems**: Emergency stop button, manual override

## Next Steps
1. ✅ Get basic system running
2. 🔧 Configure hardware interfaces  
3. 📍 Set up GPS/navigation
4. 🚜 Connect tractor controls
5. 🧭 Test autonomous navigation
6. 🛡️ Implement safety systems

## Troubleshooting
- Check `~/ros2_tractobots/build.log` for build errors
- Verify all dependencies with `rosdep check --from-paths src --ignore-src`
- Test individual nodes: `ros2 node list`, `ros2 topic list`
