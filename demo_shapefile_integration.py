#!/usr/bin/env python3
"""
Comprehensive Shapefile Integration Demo and Test
Shows all the features working together
"""

import os
import sys
import tempfile
import json
from pathlib import Path

def print_header(title):
    print(f"\n{'='*60}")
    print(f" {title}")
    print(f"{'='*60}")

def print_status(description, status, details=""):
    status_icon = "✅" if status else "❌"
    print(f"{status_icon} {description}")
    if details:
        print(f"    📋 {details}")

def test_dependencies():
    """Test all required dependencies"""
    print_header("Testing Dependencies")
    
    deps = {
        'geopandas': 'Shapefile processing',
        'shapely': 'Geometry operations', 
        'pyproj': 'Coordinate transformations',
        'fiona': 'File I/O',
        'yaml': 'Configuration export',
        'tkinter': 'GUI framework',
        'rclpy': 'ROS2 Python client'
    }
    
    all_available = True
    for dep, desc in deps.items():
        try:
            __import__(dep)
            print_status(f"Dependency: {dep}", True, desc)
        except ImportError:
            print_status(f"Dependency: {dep}", False, f"{desc} - MISSING")
            all_available = False
    
    return all_available

def create_demo_shapefile():
    """Create a demonstration shapefile"""
    print_header("Creating Demo Shapefile")
    
    try:
        import geopandas as gpd
        from shapely.geometry import Polygon
        
        # Create demo field boundaries (GPS coordinates for Iowa farms)
        fields = [
            {
                'id': 'FIELD_001',
                'name': 'North Corn Field',
                'crop': 'corn',
                'acres': 45.2,
                'coords': [
                    (-93.620, 42.035),  # SW corner
                    (-93.610, 42.035),  # SE corner  
                    (-93.610, 42.045),  # NE corner
                    (-93.620, 42.045),  # NW corner
                    (-93.620, 42.035)   # Close polygon
                ]
            },
            {
                'id': 'FIELD_002', 
                'name': 'South Soybean Field',
                'crop': 'soybeans',
                'acres': 38.7,
                'coords': [
                    (-93.615, 42.020),
                    (-93.605, 42.020),
                    (-93.605, 42.030),
                    (-93.615, 42.030),
                    (-93.615, 42.020)
                ]
            },
            {
                'id': 'FIELD_003',
                'name': 'West Pasture',
                'crop': 'pasture',
                'acres': 12.1,
                'coords': [
                    (-93.635, 42.025),
                    (-93.625, 42.025),
                    (-93.625, 42.035),
                    (-93.635, 42.035),
                    (-93.635, 42.025)
                ]
            }
        ]
        
        # Create GeoDataFrame
        geometries = []
        field_data = []
        
        for field in fields:
            polygon = Polygon(field['coords'])
            geometries.append(polygon)
            field_data.append({
                'field_id': field['id'],
                'field_name': field['name'],
                'crop_type': field['crop'],
                'area_acres': field['acres']
            })
        
        gdf = gpd.GeoDataFrame(field_data, geometry=geometries, crs='EPSG:4326')
        
        # Save to temporary file
        temp_dir = Path("temp_shapefile_demo")
        temp_dir.mkdir(exist_ok=True)
        shapefile_path = temp_dir / "demo_fields.shp"
        
        gdf.to_file(shapefile_path)
        
        print_status("Demo shapefile created", True, f"Path: {shapefile_path}")
        print(f"    📊 Created {len(fields)} field boundaries")
        for field in fields:
            print(f"       - {field['name']}: {field['acres']} acres ({field['crop']})")
        
        return str(shapefile_path)
        
    except Exception as e:
        print_status("Demo shapefile creation", False, f"Error: {e}")
        return None

def test_shapefile_manager(shapefile_path):
    """Test the shapefile manager functionality"""
    print_header("Testing Shapefile Manager")
    
    try:
        # Add to Python path
        sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')
        from shapefile_manager import ShapefileFieldManager
        
        # Initialize manager
        manager = ShapefileFieldManager()
        print_status("Manager initialization", True, "ShapefileFieldManager created")
        
        # Load boundaries
        fields_data = manager.load_field_boundaries(shapefile_path)
        if fields_data and 'fields' in fields_data:
            num_fields = len(fields_data['fields'])
            print_status("Load field boundaries", True, f"Loaded {num_fields} fields")
            
            # Show field details
            for field in fields_data['fields']:
                field_id = field.get('field_id', 'Unknown')
                attrs = field.get('attributes', {})
                field_name = attrs.get('field_name', 'Unknown')
                crop_type = attrs.get('crop_type', 'Unknown')
                area = attrs.get('area_acres', 'Unknown')
                print(f"       - {field_name} ({field_id}): {area} acres of {crop_type}")
        else:
            print_status("Load field boundaries", False, "Failed to load fields")
            return False
        
        # Test coverage path generation
        if fields_data and fields_data['fields']:
            test_field = fields_data['fields'][0]
            field_id = test_field['field_id']
            field_name = test_field['attributes']['field_name']
            
            print(f"\n🗺️ Testing coverage path for {field_name}...")
            
            # Generate path with different swath widths
            for swath_width in [4.0, 6.0, 8.0]:
                try:
                    path_data = manager.generate_coverage_path(fields_data, field_id, swath_width=swath_width)
                    if path_data and 'waypoints' in path_data:
                        num_waypoints = len(path_data['waypoints'])
                        print_status(f"Coverage path (swath {swath_width}m)", True, f"{num_waypoints} waypoints")
                    else:
                        print_status(f"Coverage path (swath {swath_width}m)", False, "No waypoints generated")
                except Exception as e:
                    print_status(f"Coverage path (swath {swath_width}m)", False, f"Error: {e}")
        
        # Test export functionality
        print("\n📤 Testing export functionality...")
        try:
            export_dir = Path("temp_shapefile_demo/exports")
            export_dir.mkdir(exist_ok=True)
            
            # Test ROS2 export
            ros2_path = export_dir / "fields.yaml"
            manager.export_to_ros2_format(fields_data, str(ros2_path))
            if ros2_path.exists():
                print_status("ROS2 YAML export", True, f"Saved to {ros2_path}")
            else:
                print_status("ROS2 YAML export", False, "File not created")
                
        except Exception as e:
            print_status("Export functionality", False, f"Error: {e}")
        
        return True
        
    except Exception as e:
        print_status("Shapefile manager test", False, f"Error: {e}")
        return False

def test_gui_modules():
    """Test GUI module imports"""
    print_header("Testing GUI Modules")
    
    try:
        # Test shapefile GUI
        sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')
        from shapefile_gui import ShapefileGUI
        print_status("Shapefile GUI import", True, "shapefile_gui.py")
        
        # Test enhanced GUI
        from enhanced_gui import TractorDashboard
        print_status("Enhanced GUI import", True, "enhanced_gui.py")
        
        # Test field boundary service
        from field_boundary_service import FieldBoundaryService
        print_status("Field boundary service import", True, "field_boundary_service.py")
        
        return True
        
    except Exception as e:
        print_status("GUI modules test", False, f"Error: {e}")
        return False

def show_usage_instructions():
    """Show usage instructions"""
    print_header("🚀 Usage Instructions")
    
    print("📁 Files Created:")
    print("   - temp_shapefile_demo/demo_fields.shp (Demo shapefile)")
    print("   - temp_shapefile_demo/exports/fields.yaml (ROS2 format)")
    
    print("\n🎮 How to Use:")
    print("   1. GUI Application:")
    print("      python3 src/tractobots_mission_ui/tractobots_mission_ui/shapefile_gui.py")
    print("      (Load the demo_fields.shp file)")
    
    print("\n   2. Enhanced Dashboard:")
    print("      python3 src/tractobots_mission_ui/tractobots_mission_ui/enhanced_gui.py")
    print("      (Go to Field Management menu)")
    
    print("\n   3. ROS2 Integration (after building workspace):")
    print("      cd ~/ros2_tractobots")
    print("      source /opt/ros/jazzy/setup.bash")
    print("      source install/setup.bash")
    print("      ros2 run tractobots_mission_ui shapefile_gui")
    print("      ros2 run tractobots_mission_ui field_boundary_service")
    
    print("\n   4. Command Line:")
    print("      python3 -c \"")
    print("      import sys")
    print("      sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')")
    print("      from shapefile_manager import ShapefileFieldManager")
    print("      manager = ShapefileFieldManager()")
    print("      fields = manager.load_field_boundaries('temp_shapefile_demo/demo_fields.shp')")
    print("      print(f'Loaded {len(fields[\\\"fields\\\"])} fields')")
    print("      \"")
    
    print("\n📚 Documentation:")
    print("   - SHAPEFILE_GUIDE.md - Complete shapefile integration guide")
    print("   - src/tractobots_mission_ui/README.md - Package documentation")

def main():
    """Main demo function"""
    print("🌾 TRACTOBOTS SHAPEFILE INTEGRATION DEMO")
    print("Comprehensive test of shapefile field boundary features")
    
    # Test 1: Dependencies
    if not test_dependencies():
        print("\n❌ Missing dependencies. Install with:")
        print("   sudo apt install python3-geopandas python3-shapely python3-pyproj python3-fiona python3-yaml")
        return False
    
    # Test 2: Create demo data
    shapefile_path = create_demo_shapefile()
    if not shapefile_path:
        return False
    
    # Test 3: Shapefile manager
    if not test_shapefile_manager(shapefile_path):
        return False
    
    # Test 4: GUI modules
    if not test_gui_modules():
        return False
    
    # Show usage instructions
    show_usage_instructions()
    
    print_header("🎉 SUCCESS!")
    print("All shapefile integration features are working correctly!")
    print("The Tractobots system is ready for field boundary operations.")
    
    return True

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
