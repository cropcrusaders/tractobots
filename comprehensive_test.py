#!/usr/bin/env python3
"""
🚜 COMPREHENSIVE TRACTOBOTS SYSTEM TEST
=====================================
This script tests all major components of the Tractobots system including:
- Shapefile integration
- John Deere GPS import
- ROS2 package availability
- GUI components (headless testing)
"""

import sys
import os
import subprocess
import json
from pathlib import Path

def print_header(title):
    print(f"\n{'='*50}")
    print(f"🧪 {title}")
    print(f"{'='*50}")

def print_test(test_name, success, details=""):
    status = "✅" if success else "❌"
    print(f"{status} {test_name}")
    if details:
        print(f"   {details}")

def test_basic_imports():
    """Test basic Python imports"""
    print_header("BASIC IMPORTS TEST")
    
    tests = [
        ("sys", "sys"),
        ("os", "os"),
        ("json", "json"),
        ("pathlib", "pathlib"),
    ]
    
    for name, module in tests:
        try:
            __import__(module)
            print_test(f"Import {name}", True)
        except ImportError as e:
            print_test(f"Import {name}", False, str(e))

def test_ros2_imports():
    """Test ROS2 imports"""
    print_header("ROS2 IMPORTS TEST")
    
    tests = [
        ("rclpy", "rclpy"),
        ("std_msgs", "std_msgs.msg"),
        ("geometry_msgs", "geometry_msgs.msg"),
        ("sensor_msgs", "sensor_msgs.msg"),
        ("launch", "launch"),
        ("launch_ros", "launch_ros.actions"),
    ]
    
    for name, module in tests:
        try:
            __import__(module)
            print_test(f"ROS2 {name}", True)
        except ImportError as e:
            print_test(f"ROS2 {name}", False, str(e))

def test_geospatial_imports():
    """Test geospatial library imports"""
    print_header("GEOSPATIAL LIBRARIES TEST")
    
    tests = [
        ("geopandas", "geopandas"),
        ("shapely", "shapely.geometry"),
        ("fiona", "fiona"),
        ("pyproj", "pyproj"),
    ]
    
    for name, module in tests:
        try:
            __import__(module)
            print_test(f"Geospatial {name}", True)
        except ImportError as e:
            print_test(f"Geospatial {name}", False, str(e))

def test_tractobots_modules():
    """Test Tractobots module imports"""
    print_header("TRACTOBOTS MODULES TEST")
    
    # Add src directory to Python path
    sys.path.insert(0, "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/src/tractobots_mission_ui")
    
    tests = [
        ("shapefile_manager", "tractobots_mission_ui.shapefile_manager"),
        ("field_boundary_service", "tractobots_mission_ui.field_boundary_service"),
        ("enhanced_gui", "tractobots_mission_ui.enhanced_gui"),
    ]
    
    for name, module in tests:
        try:
            __import__(module)
            print_test(f"Tractobots {name}", True)
        except ImportError as e:
            print_test(f"Tractobots {name}", False, str(e))

def test_john_deere_importer():
    """Test John Deere GPS importer"""
    print_header("JOHN DEERE GPS IMPORTER TEST")
    
    try:
        # Import the John Deere importer
        sys.path.insert(0, "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots")
        from john_deere_gps_importer import JohnDeereGPSImporter
        
        print_test("John Deere GPS Importer import", True)
        
        # Test with demo data
        demo_file = "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/demo_john_deere_data.xml"
        if os.path.exists(demo_file):
            print_test("Demo John Deere data file exists", True)
            
            # Test import
            importer = JohnDeereGPSImporter()
            result = importer.import_from_file(demo_file)
            
            if result and len(result.get('waypoints', [])) > 0:
                print_test(f"Demo data import", True, f"Found {len(result['waypoints'])} waypoints")
            else:
                print_test("Demo data import", False, "No waypoints found")
        else:
            print_test("Demo John Deere data file exists", False)
            
    except Exception as e:
        print_test("John Deere GPS Importer", False, str(e))

def test_shapefile_manager():
    """Test shapefile manager"""
    print_header("SHAPEFILE MANAGER TEST")
    
    try:
        sys.path.insert(0, "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots/src/tractobots_mission_ui")
        from tractobots_mission_ui.shapefile_manager import ShapefileFieldManager
        
        print_test("Shapefile Manager import", True)
        
        # Create manager instance
        manager = ShapefileFieldManager()
        print_test("Shapefile Manager instantiation", True)
        
        # Test creating a demo field
        demo_boundary = [
            (-102.0, 40.0),
            (-102.0, 40.01),
            (-101.99, 40.01),
            (-101.99, 40.0),
            (-102.0, 40.0)
        ]
        
        field_id = manager.add_field("Test Field", demo_boundary)
        if field_id:
            print_test("Demo field creation", True, f"Field ID: {field_id}")
        else:
            print_test("Demo field creation", False)
            
    except Exception as e:
        print_test("Shapefile Manager", False, str(e))

def test_workspace_build():
    """Test ROS2 workspace build status"""
    print_header("ROS2 WORKSPACE TEST")
    
    workspace_dir = "/home/bass/ros2_tractobots"
    
    # Check if workspace exists
    if os.path.exists(workspace_dir):
        print_test("Workspace directory exists", True, workspace_dir)
        
        # Check if built
        install_dir = os.path.join(workspace_dir, "install")
        if os.path.exists(install_dir):
            print_test("Workspace install directory exists", True)
            
            # Check for tractobots packages
            tractobots_pkg = os.path.join(install_dir, "tractobots_mission_ui")
            if os.path.exists(tractobots_pkg):
                print_test("Tractobots mission UI package installed", True)
            else:
                print_test("Tractobots mission UI package installed", False)
        else:
            print_test("Workspace install directory exists", False)
    else:
        print_test("Workspace directory exists", False)

def test_files_exist():
    """Test if key files exist"""
    print_header("FILE EXISTENCE TEST")
    
    base_path = "/mnt/c/Users/nicholas/OneDrive/Documents/GitHub/tractobots"
    
    files_to_check = [
        "john_deere_gps_importer.py",
        "john_deere_gps_gui.py",
        "demo_john_deere_data.xml",
        "tractobots_desktop_app.py",
        "src/tractobots_mission_ui/tractobots_mission_ui/shapefile_manager.py",
        "src/tractobots_mission_ui/tractobots_mission_ui/field_boundary_service.py",
        "src/tractobots_mission_ui/tractobots_mission_ui/enhanced_gui.py",
        "SHAPEFILE_GUIDE.md",
        "JOHN_DEERE_GPS_GUIDE.md",
    ]
    
    for file_path in files_to_check:
        full_path = os.path.join(base_path, file_path)
        exists = os.path.exists(full_path)
        print_test(f"File: {file_path}", exists)

def main():
    """Run all tests"""
    print("🚜 TRACTOBOTS COMPREHENSIVE SYSTEM TEST")
    print("======================================")
    print("Testing all major components...")
    
    test_basic_imports()
    test_ros2_imports()
    test_geospatial_imports()
    test_tractobots_modules()
    test_john_deere_importer()
    test_shapefile_manager()
    test_workspace_build()
    test_files_exist()
    
    print_header("TEST COMPLETE")
    print("🎉 Comprehensive test finished!")
    print("📋 Review the results above to identify any issues.")
    print("✅ Items marked with ✅ are working correctly")
    print("❌ Items marked with ❌ need attention")

if __name__ == "__main__":
    main()
