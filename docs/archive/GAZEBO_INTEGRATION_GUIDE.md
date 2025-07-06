# 🚜 Gazebo Professional 3D Simulator for Tractobots

## Overview

This document provides complete setup and usage instructions for integrating Gazebo Garden with the Tractobots ROS2 system. Gazebo provides a professional-grade 3D simulation environment specifically designed for robotics applications, making it ideal for agricultural robotics testing and development.

## Why Gazebo for Tractobots?

### ✅ **Professional Features**
- **Realistic Physics**: Accurate physics simulation for agricultural equipment
- **Sensor Simulation**: GPS, IMU, LiDAR, cameras, and agricultural sensors
- **Weather Simulation**: Wind, rain, and lighting conditions
- **Terrain Modeling**: Realistic field surfaces and obstacles
- **ROS2 Integration**: Native ROS2 support with message bridges

### ✅ **Agricultural Advantages**
- **Field Environments**: Create realistic farm fields and orchards
- **Crop Simulation**: Model various crop types and growth stages
- **Obstacle Simulation**: Trees, fences, buildings, and equipment
- **Path Planning**: Test coverage paths and GPS guidance lines
- **Multi-Robot**: Simulate multiple tractors and implements

## Installation

### Quick Setup
Run the automated installation script:
```bash
chmod +x setup_gazebo_simulator.sh
./setup_gazebo_simulator.sh
```

### Manual Installation
If you prefer manual installation:

1. **Install Gazebo Garden for ROS2 Jazzy:**
```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

2. **Install Additional Packages:**
```bash
sudo apt install -y gz-garden gz-sim7 gz-tools2 ros-jazzy-nav2-bringup
```

3. **Set Environment Variables:**
```bash
echo "export GZ_SIM_RESOURCE_PATH=~/.gazebo/models:/usr/share/gazebo-11/models" >> ~/.bashrc
source ~/.bashrc
```

## Usage

### 1. Launch Gazebo Farm Simulation

**Basic Launch:**
```bash
# Source both ROS2 and Gazebo workspace
source /opt/ros/jazzy/setup.bash
source ~/tractobots_gazebo_ws/install/setup.bash

# Launch the farm simulation
ros2 launch tractobots_gazebo tractobots_gazebo.launch.py
```

**Launch with Custom World:**
```bash
ros2 launch tractobots_gazebo tractobots_gazebo.launch.py world:=path/to/your/world.sdf
```

### 2. Launch RViz for Visualization

In a new terminal:
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

### 3. Test Direct Gazebo

Launch Gazebo directly to test the installation:
```bash
gz sim ~/tractobots_gazebo_ws/src/tractobots_gazebo/worlds/farm_field.sdf
```

## World Files

### Farm Field World (farm_field.sdf)
- **Size**: 100m x 100m field
- **Features**: Ground plane, realistic lighting, wind simulation
- **Purpose**: Basic field operations testing

### Creating Custom Worlds

1. **Create a new world file:**
```bash
mkdir -p ~/tractobots_gazebo_ws/src/tractobots_gazebo/worlds
nano ~/tractobots_gazebo_ws/src/tractobots_gazebo/worlds/orchard.sdf
```

2. **Example Orchard World:**
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="orchard">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Tree rows for orchard simulation -->
    <model name="tree_row_1">
      <pose>-40 -20 0 0 0 0</pose>
      <!-- Tree models would go here -->
    </model>
    
    <!-- Add more trees, obstacles, etc. -->
  </world>
</sdf>
```

## ROS2 Integration

### Message Bridges
The system automatically bridges these topics between ROS2 and Gazebo:

| ROS2 Topic | Gazebo Topic | Message Type |
|------------|--------------|--------------|
| `/clock` | `/clock` | Clock |
| `/tf` | `/tf` | TFMessage |
| `/odom` | `/odom` | Odometry |
| `/scan` | `/scan` | LaserScan |
| `/imu` | `/imu` | IMU |
| `/gps` | `/gps` | NavSatFix |
| `/cmd_vel` | `/cmd_vel` | Twist |

### Custom Topic Bridges
Add custom bridges in the launch file:
```python
# In tractobots_gazebo.launch.py
ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/your_topic@your_msg_type[ignition.msgs.YourType',
        # Add more topic bridges here
    ],
    output='screen'
)
```

## Importing Tractobots Field Data

### 1. Shapefile Integration

Convert your shapefile field boundaries to Gazebo world coordinates:

```python
# In your shapefile processing code
def convert_to_gazebo_coordinates(shapefile_data):
    """Convert shapefile coordinates to Gazebo world coordinates"""
    # Your existing shapefile processing code
    field_boundaries = process_shapefile(shapefile_data)
    
    # Convert to Gazebo SDF format
    gazebo_world = generate_gazebo_world(field_boundaries)
    
    # Save as world file
    with open('custom_field.sdf', 'w') as f:
        f.write(gazebo_world)
    
    return gazebo_world
```

### 2. John Deere GPS Lines

Import GPS guidance lines as waypoints:

```python
# In your John Deere GPS processing code
def export_to_gazebo_waypoints(gps_lines):
    """Export GPS lines as Gazebo waypoints"""
    waypoints = []
    for line in gps_lines:
        for point in line.points:
            waypoints.append({
                'x': point.longitude,
                'y': point.latitude,
                'z': 0.0
            })
    
    # Create waypoint markers in Gazebo
    create_gazebo_waypoints(waypoints)
    
    return waypoints
```

### 3. Navigation Integration

Connect Nav2 with Gazebo simulation:

```bash
# Launch Nav2 with Gazebo
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True

# Set initial pose
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "..."

# Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "..."
```

## Testing and Validation

### 1. Basic System Test

```bash
# Terminal 1: Launch Gazebo
ros2 launch tractobots_gazebo tractobots_gazebo.launch.py

# Terminal 2: Check topics
ros2 topic list

# Terminal 3: Test navigation
ros2 run nav2_simple_commander nav2_simple_commander
```

### 2. Field Boundary Test

```bash
# Import your shapefile
python3 -c "
import sys
sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')
from shapefile_manager import ShapefileManager
manager = ShapefileManager()
manager.import_shapefile('your_field.shp')
manager.export_to_gazebo_world('field_world.sdf')
"

# Launch with custom field
ros2 launch tractobots_gazebo tractobots_gazebo.launch.py world:=field_world.sdf
```

### 3. GPS Guidance Test

```bash
# Import John Deere GPS lines
python3 john_deere_gps_importer.py --input your_gps_lines.xml --output gazebo_waypoints.yaml

# Launch navigation with GPS waypoints
ros2 launch nav2_bringup navigation_launch.py params_file:=gazebo_waypoints.yaml
```

## Advanced Features

### 1. Multi-Robot Simulation

```xml
<!-- In your world file -->
<model name="tractor_1">
  <pose>0 0 0 0 0 0</pose>
  <!-- Tractor model -->
</model>

<model name="tractor_2">
  <pose>10 0 0 0 0 0</pose>
  <!-- Second tractor model -->
</model>
```

### 2. Sensor Simulation

```xml
<!-- Add sensors to your robot model -->
<sensor name="gps" type="gps">
  <pose>0 0 1 0 0 0</pose>
  <gps>
    <horizontal_noise>0.1</horizontal_noise>
    <vertical_noise>0.1</vertical_noise>
  </gps>
</sensor>

<sensor name="imu" type="imu">
  <pose>0 0 0.5 0 0 0</pose>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.009</stddev>
        </noise>
      </x>
    </angular_velocity>
  </imu>
</sensor>
```

### 3. Weather Simulation

```xml
<!-- Add weather effects -->
<world name="farm_with_weather">
  <!-- Wind simulation -->
  <wind>
    <linear_velocity>5 2 0</linear_velocity>
  </wind>
  
  <!-- Fog/visibility -->
  <scene>
    <fog>
      <color>0.8 0.8 0.8</color>
      <type>linear</type>
      <start>10</start>
      <end>100</end>
      <density>0.5</density>
    </fog>
  </scene>
</world>
```

## Troubleshooting

### Common Issues

1. **Gazebo Won't Start**
```bash
# Check installation
gz sim --version

# Check environment variables
echo $GZ_SIM_RESOURCE_PATH

# Reset Gazebo
rm -rf ~/.gazebo/log
```

2. **ROS2-Gazebo Bridge Issues**
```bash
# Check bridge topics
ros2 topic list | grep -E "(clock|tf|odom)"

# Restart bridge
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
```

3. **Performance Issues**
```bash
# Reduce physics update rate
export GZ_SIM_PHYSICS_MAX_STEP_SIZE=0.01

# Disable shadows
export GZ_SIM_RENDER_SHADOWS=false
```

### Debug Commands

```bash
# Check Gazebo logs
gz log

# Monitor ROS2 topics
ros2 topic echo /clock
ros2 topic echo /tf

# Check transform tree
ros2 run tf2_tools view_frames
```

## Best Practices

### 1. World Design
- Keep models simple for better performance
- Use LOD (Level of Detail) models for distant objects
- Optimize mesh complexity for real-time simulation

### 2. Sensor Configuration
- Match sensor noise to real-world specifications
- Use appropriate update rates for different sensors
- Consider computational load of multiple sensors

### 3. Performance Optimization
- Use static collision meshes where possible
- Limit physics simulation complexity
- Monitor CPU and memory usage

### 4. Testing Workflow
1. Start with simple worlds
2. Add complexity gradually
3. Test each component individually
4. Validate against real-world data

## Integration with Tractobots GUI

### Launch from Desktop App

Add Gazebo integration to your PyQt5 desktop application:

```python
# In qt_gui.py
def launch_gazebo_simulator(self):
    """Launch Gazebo farm simulation"""
    try:
        # Source environment and launch
        cmd = [
            'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && '
            'source ~/tractobots_gazebo_ws/install/setup.bash && '
            'ros2 launch tractobots_gazebo tractobots_gazebo.launch.py'
        ]
        
        self.gazebo_process = subprocess.Popen(cmd)
        self.status_label.setText("Gazebo simulator started")
        
    except Exception as e:
        self.status_label.setText(f"Error starting Gazebo: {str(e)}")
```

### Field Visualization

Integrate field boundaries with Gazebo visualization:

```python
# In shapefile_manager.py
def export_to_gazebo_world(self, output_file):
    """Export current field boundaries to Gazebo world file"""
    if not self.field_boundaries:
        raise ValueError("No field boundaries loaded")
    
    # Generate SDF world file
    sdf_content = self.generate_sdf_world()
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    
    return output_file
```

## Next Steps

1. **Run the setup script**: `./setup_gazebo_simulator.sh`
2. **Test the installation**: Launch the basic farm simulation
3. **Import your field data**: Use shapefile and GPS import tools
4. **Customize worlds**: Create specific field layouts for your use case
5. **Integrate with navigation**: Test path planning and execution

## Support

For issues or questions:
- Check the troubleshooting section above
- Review Gazebo documentation: https://gazebosim.org/docs
- Check ROS2-Gazebo integration: https://github.com/gazebosim/ros_gz

Your professional 3D agricultural robotics simulator is now ready! 🚜🌾
