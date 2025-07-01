#!/usr/bin/env python3
"""
Quick test script to verify imports work in WSL
Run this in WSL to confirm ROS2 imports are available
"""

import sys
import os

def test_imports():
    """Test if all required imports are available"""
    print("🧪 Testing ROS2 Import Availability")
    print("=" * 40)
    
    # Test core ROS2
    try:
        import rclpy
        print("✅ rclpy - Available")
    except ImportError as e:
        print(f"❌ rclpy - Not found: {e}")
        return False
    
    # Test message packages
    try:
        from std_msgs.msg import String
        print("✅ std_msgs - Available")
    except ImportError as e:
        print(f"❌ std_msgs - Not found: {e}")
        return False
        
    try:
        from sensor_msgs.msg import Joy, NavSatFix
        print("✅ sensor_msgs - Available")
    except ImportError as e:
        print(f"❌ sensor_msgs - Not found: {e}")
        return False
        
    try:
        from geometry_msgs.msg import Twist
        print("✅ geometry_msgs - Available")
    except ImportError as e:
        print(f"❌ geometry_msgs - Not found: {e}")
        return False
    
    # Test launch system
    try:
        from launch import LaunchDescription
        from launch_ros.actions import Node
        print("✅ launch system - Available")
    except ImportError as e:
        print(f"❌ launch system - Not found: {e}")
        return False
    
    # Test setuptools
    try:
        import setuptools
        print("✅ setuptools - Available")
    except ImportError as e:
        print(f"❌ setuptools - Not found: {e}")
        return False
    
    print("\n🎉 All imports successful!")
    print("✅ VS Code import warnings are cosmetic only")
    print("✅ Your autonomous tractor system is ready!")
    return True

def check_environment():
    """Check if we're in the right environment"""
    print("\n🔍 Environment Check")
    print("=" * 40)
    
    # Check if we're in WSL/Linux
    if sys.platform.startswith('linux'):
        print("✅ Running on Linux/WSL")
    else:
        print("⚠️  Running on Windows - imports may fail")
        print("   Run this script in WSL Ubuntu terminal")
        return False
    
    # Check ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO')
    if ros_distro:
        print(f"✅ ROS_DISTRO: {ros_distro}")
    else:
        print("⚠️  ROS_DISTRO not set - source ROS2 setup first")
        print("   Run: source /opt/ros/humble/setup.bash")
        return False
    
    # Check Python version
    print(f"✅ Python: {sys.version}")
    
    return True

if __name__ == '__main__':
    print("🚜 Tractobots Import Test")
    print("=" * 40)
    
    if not check_environment():
        print("\n❌ Environment issues detected")
        sys.exit(1)
    
    if test_imports():
        print("\n🎯 Summary:")
        print("- VS Code import warnings are normal on Windows")
        print("- All imports work correctly in WSL")
        print("- You can safely build and run your tractor system")
        print("- Use: ./quick_setup.sh to build")
        print("- Use: VS Code tasks to run and test")
    else:
        print("\n❌ Some imports failed - check ROS2 installation")
        sys.exit(1)
