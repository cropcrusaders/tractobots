#!/bin/bash
# =============================================================================
# Simple Gazebo Integration Test for Tractobots
# =============================================================================

echo "🚜 Testing Gazebo Integration for Tractobots 🚜"
echo "================================================"

# Test 1: Check if Gazebo is installed
echo "✅ Test 1: Gazebo Installation"
if command -v gz &> /dev/null; then
    echo "   ✓ Gazebo command found"
    gz sim --version | head -1
else
    echo "   ✗ Gazebo command not found"
fi

# Test 2: Check workspace
echo ""
echo "✅ Test 2: Workspace Structure"
if [ -d "$HOME/tractobots_gazebo_ws" ]; then
    echo "   ✓ Workspace directory exists"
    
    if [ -f "$HOME/tractobots_gazebo_ws/src/tractobots_gazebo/package.xml" ]; then
        echo "   ✓ Package.xml found"
    else
        echo "   ✗ Package.xml missing"
    fi
    
    if [ -f "$HOME/tractobots_gazebo_ws/src/tractobots_gazebo/worlds/farm_field.sdf" ]; then
        echo "   ✓ Farm field world found"
    else
        echo "   ✗ Farm field world missing"
    fi
    
    if [ -f "$HOME/tractobots_gazebo_ws/src/tractobots_gazebo/launch/tractobots_gazebo.launch.py" ]; then
        echo "   ✓ Launch file found"
    else
        echo "   ✗ Launch file missing"
    fi
else
    echo "   ✗ Workspace directory not found"
fi

# Test 3: Check ROS2 
echo ""
echo "✅ Test 3: ROS2 Integration"
if command -v ros2 &> /dev/null; then
    echo "   ✓ ROS2 command found"
    ros2 --version
else
    echo "   ✗ ROS2 command not found"
fi

# Test 4: Check Python modules
echo ""
echo "✅ Test 4: Python Integration"
python3 -c "
try:
    import sys
    sys.path.insert(0, 'src/tractobots_mission_ui/tractobots_mission_ui')
    from shapefile_manager import ShapefileManager
    manager = ShapefileManager()
    if hasattr(manager, 'export_to_gazebo_world'):
        print('   ✓ Shapefile Gazebo export available')
    else:
        print('   ✗ Shapefile Gazebo export missing')
except Exception as e:
    print(f'   ✗ Shapefile manager error: {e}')

try:
    from john_deere_gps_importer import JohnDeereGPSImporter
    importer = JohnDeereGPSImporter()
    if hasattr(importer, 'export_to_gazebo_world'):
        print('   ✓ John Deere GPS Gazebo export available')
    else:
        print('   ✗ John Deere GPS Gazebo export missing')
except Exception as e:
    print(f'   ✗ John Deere GPS importer error: {e}')
"

# Test 5: System resources
echo ""
echo "✅ Test 5: System Resources"
echo "   CPU cores: $(nproc)"
echo "   Memory: $(free -h | grep Mem | awk '{print $2}')"
echo "   Disk space: $(df -h ~ | tail -1 | awk '{print $4}')"

echo ""
echo "================================================"
echo "🎉 Gazebo Integration Test Complete!"
echo ""
echo "To launch Gazebo simulation:"
echo "  1. cd ~/tractobots_gazebo_ws"
echo "  2. source /opt/ros/jazzy/setup.bash"
echo "  3. source install/setup.bash"
echo "  4. ros2 launch tractobots_gazebo tractobots_gazebo.launch.py"
echo ""
echo "To launch just the world file:"
echo "  gz sim src/tractobots_gazebo/worlds/farm_field.sdf"
echo ""
echo "Happy simulating! 🌾"
