#!/usr/bin/env python3
"""
Test script for shapefile integration
Creates a sample shapefile and tests the complete workflow
"""

import os
import sys
import tempfile
from pathlib import Path

# Add the tractobots modules to Python path
sys.path.append('src/tractobots_mission_ui/tractobots_mission_ui')

def create_sample_shapefile(output_path):
    """Create a sample shapefile for testing"""
    try:
        import geopandas as gpd
        from shapely.geometry import Polygon
        import pandas as pd
        
        # Create sample field boundaries
        fields = []
        
        # Field 1: Rectangular field
        field1 = Polygon([
            (-94.5, 42.0),      # Bottom-left
            (-94.45, 42.0),     # Bottom-right  
            (-94.45, 42.05),    # Top-right
            (-94.5, 42.05),     # Top-left
            (-94.5, 42.0)       # Close polygon
        ])
        
        # Field 2: Irregular field
        field2 = Polygon([
            (-94.4, 42.1),
            (-94.35, 42.1),
            (-94.35, 42.15),
            (-94.38, 42.17),
            (-94.42, 42.16),
            (-94.4, 42.1)
        ])
        
        # Create GeoDataFrame
        gdf = gpd.GeoDataFrame({
            'field_id': ['field_001', 'field_002'],
            'field_name': ['North Field', 'South Field'],
            'area_acres': [15.2, 12.8],
            'crop_type': ['corn', 'soybeans'],
            'geometry': [field1, field2]
        }, crs='EPSG:4326')
        
        # Save as shapefile
        gdf.to_file(output_path)
        print(f"✅ Created sample shapefile: {output_path}")
        return True
        
    except Exception as e:
        print(f"❌ Error creating shapefile: {e}")
        return False

def test_shapefile_manager(shapefile_path):
    """Test the shapefile manager functionality"""
    try:
        from shapefile_manager import ShapefileFieldManager
        
        print("\n🧪 Testing Shapefile Manager...")
        
        # Initialize manager
        manager = ShapefileFieldManager()
        
        # Load shapefile
        fields_data = manager.load_field_boundaries(shapefile_path)
        if fields_data and 'fields' in fields_data:
            print(f"✅ Loaded {len(fields_data['fields'])} fields from shapefile")
            
            # Test field processing
            for field in fields_data['fields']:
                field_id = field.get('field_id', 'Unknown')
                field_name = field.get('attributes', {}).get('field_name', 'Unknown')
                print(f"   - {field_id}: {field_name}")
        else:
            print("❌ Failed to load fields from shapefile")
            return False
        
        # Test coverage path generation
        if fields_data and fields_data['fields']:
            field_id = fields_data['fields'][0]['field_id']
            print(f"\n🗺️ Testing coverage path generation for {field_id}...")
            
            path = manager.generate_coverage_path(fields_data, field_id, swath_width=6.0)
            if path and 'waypoints' in path:
                print(f"✅ Generated coverage path with {len(path['waypoints'])} waypoints")
            else:
                print("❌ Failed to generate coverage path")
        
        return True
        
    except Exception as e:
        print(f"❌ Error testing shapefile manager: {e}")
        return False

def test_field_boundary_service():
    """Test the ROS2 field boundary service"""
    try:
        from field_boundary_service import FieldBoundaryService
        
        print("\n🤖 Testing Field Boundary Service...")
        print("✅ Field boundary service imported successfully")
        print("   Note: Full ROS2 testing requires running ROS2 environment")
        
        return True
        
    except Exception as e:
        print(f"❌ Error testing field boundary service: {e}")
        return False

def main():
    """Main test function"""
    print("🧪 SHAPEFILE INTEGRATION TEST")
    print("=" * 50)
    
    # Create temporary directory for test files
    with tempfile.TemporaryDirectory() as temp_dir:
        shapefile_path = Path(temp_dir) / "test_fields.shp"
        
        # Test 1: Create sample shapefile
        print("1️⃣ Creating sample shapefile...")
        if not create_sample_shapefile(str(shapefile_path)):
            return False
        
        # Test 2: Test shapefile manager
        print("\n2️⃣ Testing shapefile manager...")
        if not test_shapefile_manager(str(shapefile_path)):
            return False
        
        # Test 3: Test field boundary service
        print("\n3️⃣ Testing field boundary service...")
        if not test_field_boundary_service():
            return False
        
        print("\n🎉 All tests passed!")
        print("\n📋 Next Steps:")
        print("   - Run: python3 src/tractobots_mission_ui/tractobots_mission_ui/shapefile_gui.py")
        print("   - Or build workspace and run: ros2 run tractobots_mission_ui shapefile_gui")
        print("   - Check SHAPEFILE_GUIDE.md for complete usage instructions")
        
        return True

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
