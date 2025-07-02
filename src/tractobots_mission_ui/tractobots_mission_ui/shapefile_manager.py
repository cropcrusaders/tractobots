#!/usr/bin/env python3
"""
Shapefile Field Boundary Manager for Tractobots
Handles .shp files from operation centers for field boundary definition
"""

import os
import sys
import json
import numpy as np
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ShapefileFieldManager:
    """
    Manages shapefile integration for field boundaries in Tractobots system
    """
    
    def __init__(self, shapefiles_dir: str = "field_data/shapefiles"):
        self.shapefiles_dir = Path(shapefiles_dir)
        self.shapefiles_dir.mkdir(parents=True, exist_ok=True)
        
        # Try to import required libraries
        self.geopandas_available = self._check_geopandas()
        self.shapely_available = self._check_shapely()
        
        if not (self.geopandas_available and self.shapely_available):
            logger.warning("GeoPandas or Shapely not available. Install with: pip install geopandas shapely")
    
    def _check_geopandas(self) -> bool:
        """Check if geopandas is available"""
        try:
            import geopandas as gpd
            return True
        except ImportError:
            return False
    
    def _check_shapely(self) -> bool:
        """Check if shapely is available"""
        try:
            import shapely
            return True
        except ImportError:
            return False
    
    def load_field_boundaries(self, shapefile_path: str) -> Optional[Dict]:
        """
        Load field boundaries from shapefile
        
        Args:
            shapefile_path: Path to the .shp file
            
        Returns:
            Dictionary containing field boundary data or None if failed
        """
        if not self.geopandas_available:
            logger.error("GeoPandas required for shapefile processing. Install with: pip install geopandas")
            return None
            
        try:
            import geopandas as gpd
            from shapely.geometry import Point, Polygon
            
            # Load shapefile
            gdf = gpd.read_file(shapefile_path)
            
            # Convert to WGS84 if not already
            if gdf.crs != 'EPSG:4326':
                gdf = gdf.to_crs('EPSG:4326')
            
            fields_data = {
                'fields': [],
                'metadata': {
                    'source_file': str(shapefile_path),
                    'crs': str(gdf.crs),
                    'num_fields': len(gdf)
                }
            }
            
            for idx, row in gdf.iterrows():
                field_data = {
                    'field_id': f"field_{idx}",
                    'geometry_type': str(row.geometry.geom_type),
                    'attributes': {}
                }
                
                # Extract field attributes
                for col in gdf.columns:
                    if col != 'geometry':
                        field_data['attributes'][col] = str(row[col]) if row[col] is not None else ""
                
                # Extract boundary coordinates
                if row.geometry.geom_type == 'Polygon':
                    coords = list(row.geometry.exterior.coords)
                    field_data['boundary_points'] = [{'lat': lat, 'lon': lon} for lon, lat in coords]
                    
                    # Calculate field center
                    centroid = row.geometry.centroid
                    field_data['center'] = {'lat': centroid.y, 'lon': centroid.x}
                    
                    # Calculate approximate area in hectares
                    # Project to equal area projection for accurate area calculation
                    utm_gdf = gpd.GeoDataFrame([row], crs='EPSG:4326').to_crs('EPSG:3857')
                    area_m2 = utm_gdf.geometry.area.iloc[0]
                    field_data['area_hectares'] = area_m2 / 10000  # Convert to hectares
                    
                elif row.geometry.geom_type == 'MultiPolygon':
                    # Handle multiple polygons (complex fields)
                    field_data['boundary_points'] = []
                    for polygon in row.geometry.geoms:
                        coords = list(polygon.exterior.coords)
                        field_data['boundary_points'].append([{'lat': lat, 'lon': lon} for lon, lat in coords])
                    
                    centroid = row.geometry.centroid
                    field_data['center'] = {'lat': centroid.y, 'lon': centroid.x}
                    
                    # Calculate total area
                    utm_gdf = gpd.GeoDataFrame([row], crs='EPSG:4326').to_crs('EPSG:3857')
                    area_m2 = utm_gdf.geometry.area.iloc[0]
                    field_data['area_hectares'] = area_m2 / 10000
                
                fields_data['fields'].append(field_data)
            
            logger.info(f"Successfully loaded {len(fields_data['fields'])} fields from {shapefile_path}")
            return fields_data
            
        except Exception as e:
            logger.error(f"Error loading shapefile {shapefile_path}: {e}")
            return None
    
    def save_field_data(self, fields_data: Dict, output_file: str = "field_boundaries.json") -> bool:
        """
        Save field boundary data to JSON file for use by ROS2 nodes
        
        Args:
            fields_data: Field boundary data dictionary
            output_file: Output JSON filename
            
        Returns:
            True if successful, False otherwise
        """
        try:
            output_path = self.shapefiles_dir / output_file
            with open(output_path, 'w') as f:
                json.dump(fields_data, f, indent=2)
            
            logger.info(f"Field data saved to {output_path}")
            return True
            
        except Exception as e:
            logger.error(f"Error saving field data: {e}")
            return False
    
    def generate_ros2_field_config(self, fields_data: Dict, output_file: str = "field_config.yaml") -> bool:
        """
        Generate ROS2 compatible field configuration
        
        Args:
            fields_data: Field boundary data dictionary
            output_file: Output YAML filename
            
        Returns:
            True if successful, False otherwise
        """
        try:
            import yaml
            
            config = {
                'field_boundaries': {
                    'metadata': fields_data['metadata'],
                    'fields': {}
                }
            }
            
            for field in fields_data['fields']:
                field_id = field['field_id']
                config['field_boundaries']['fields'][field_id] = {
                    'name': field['attributes'].get('NAME', field_id),
                    'area_hectares': field.get('area_hectares', 0.0),
                    'center_lat': field['center']['lat'],
                    'center_lon': field['center']['lon'],
                    'boundary_coordinates': field['boundary_points'],
                    'attributes': field['attributes']
                }
            
            output_path = self.shapefiles_dir / output_file
            with open(output_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)
            
            logger.info(f"ROS2 field config saved to {output_path}")
            return True
            
        except ImportError:
            logger.error("PyYAML required for YAML output. Install with: pip install PyYAML")
            return False
        except Exception as e:
            logger.error(f"Error generating ROS2 config: {e}")
            return False
    
    def convert_to_geofence(self, fields_data: Dict, field_id: str) -> Optional[List[Tuple[float, float]]]:
        """
        Convert field boundary to simple lat/lon geofence for navigation
        
        Args:
            fields_data: Field boundary data dictionary
            field_id: ID of the field to convert
            
        Returns:
            List of (lat, lon) tuples or None if not found
        """
        try:
            field_data = None
            for field in fields_data['fields']:
                if field['field_id'] == field_id:
                    field_data = field
                    break
            
            if not field_data:
                logger.error(f"Field {field_id} not found")
                return None
            
            if isinstance(field_data['boundary_points'][0], list):
                # MultiPolygon - use first polygon
                boundary_points = field_data['boundary_points'][0]
            else:
                # Single polygon
                boundary_points = field_data['boundary_points']
            
            geofence = [(point['lat'], point['lon']) for point in boundary_points]
            logger.info(f"Generated geofence for {field_id} with {len(geofence)} points")
            return geofence
            
        except Exception as e:
            logger.error(f"Error converting field to geofence: {e}")
            return None
    
    def generate_coverage_path(self, fields_data: Dict, field_id: str, 
                             swath_width: float = 5.0, overlap: float = 0.1) -> Optional[List[Tuple[float, float]]]:
        """
        Generate basic coverage path for field
        
        Args:
            fields_data: Field boundary data dictionary
            field_id: ID of the field
            swath_width: Width of implement in meters
            overlap: Overlap percentage (0.1 = 10%)
            
        Returns:
            List of (lat, lon) waypoints for coverage path
        """
        if not self.shapely_available:
            logger.error("Shapely required for path generation. Install with: pip install shapely")
            return None
            
        try:
            from shapely.geometry import Polygon, Point
            import pyproj
            
            # Find field
            field_data = None
            for field in fields_data['fields']:
                if field['field_id'] == field_id:
                    field_data = field
                    break
            
            if not field_data:
                logger.error(f"Field {field_id} not found")
                return None
            
            # Get boundary points
            if isinstance(field_data['boundary_points'][0], list):
                boundary_points = field_data['boundary_points'][0]
            else:
                boundary_points = field_data['boundary_points']
            
            # Create polygon
            coords = [(point['lon'], point['lat']) for point in boundary_points]
            polygon = Polygon(coords)
            
            # Get bounding box
            minx, miny, maxx, maxy = polygon.bounds
            
            # Simple back-and-forth pattern
            # This is a basic implementation - for production use libraries like coverage_path_planning
            waypoints = []
            
            # Calculate effective swath width with overlap
            effective_swath = swath_width * (1 - overlap)
            
            # Convert degrees to approximate meters (rough conversion)
            lat_center = (miny + maxy) / 2
            meters_per_degree_lat = 111000
            meters_per_degree_lon = 111000 * np.cos(np.radians(lat_center))
            
            # Swath spacing in degrees
            swath_spacing_lat = effective_swath / meters_per_degree_lat
            
            # Generate parallel lines
            current_lat = miny + swath_spacing_lat
            direction = 1  # 1 for left-to-right, -1 for right-to-left
            
            while current_lat < maxy - swath_spacing_lat:
                if direction == 1:
                    # Left to right
                    start_lon = minx
                    end_lon = maxx
                else:
                    # Right to left
                    start_lon = maxx
                    end_lon = minx
                
                # Check if line intersects with field
                start_point = Point(start_lon, current_lat)
                end_point = Point(end_lon, current_lat)
                
                if polygon.contains(start_point) or polygon.intersects(start_point):
                    waypoints.append((current_lat, start_lon))
                
                if polygon.contains(end_point) or polygon.intersects(end_point):
                    waypoints.append((current_lat, end_lon))
                
                current_lat += swath_spacing_lat
                direction *= -1  # Alternate direction
            
            logger.info(f"Generated {len(waypoints)} waypoints for field {field_id}")
            return waypoints
            
        except Exception as e:
            logger.error(f"Error generating coverage path: {e}")
            return None

def install_dependencies():
    """Install required dependencies for shapefile processing"""
    try:
        import subprocess
        
        packages = [
            'geopandas',
            'shapely',
            'pyproj',
            'fiona',
            'PyYAML'
        ]
        
        print("Installing shapefile processing dependencies...")
        for package in packages:
            print(f"Installing {package}...")
            subprocess.check_call([sys.executable, '-m', 'pip', 'install', '--user', package])
        
        print("✅ All dependencies installed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Error installing dependencies: {e}")
        print("Try manual installation: pip install geopandas shapely pyproj fiona PyYAML")
        return False

def main():
    """Example usage of ShapefileFieldManager"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Tractobots Shapefile Field Manager')
    parser.add_argument('--install-deps', action='store_true', 
                       help='Install required dependencies')
    parser.add_argument('--shapefile', type=str, 
                       help='Path to shapefile to process')
    parser.add_argument('--field-id', type=str, 
                       help='Field ID for path generation')
    parser.add_argument('--swath-width', type=float, default=5.0,
                       help='Implement width in meters (default: 5.0)')
    
    args = parser.parse_args()
    
    if args.install_deps:
        install_dependencies()
        return
    
    if not args.shapefile:
        print("Usage examples:")
        print("  # Install dependencies")
        print("  python3 shapefile_manager.py --install-deps")
        print()
        print("  # Process shapefile")
        print("  python3 shapefile_manager.py --shapefile /path/to/fields.shp")
        print()
        print("  # Generate coverage path")
        print("  python3 shapefile_manager.py --shapefile /path/to/fields.shp --field-id field_0 --swath-width 6.0")
        return
    
    # Initialize manager
    manager = ShapefileFieldManager()
    
    # Load shapefile
    print(f"Loading shapefile: {args.shapefile}")
    fields_data = manager.load_field_boundaries(args.shapefile)
    
    if not fields_data:
        print("❌ Failed to load shapefile")
        return
    
    # Save field data
    manager.save_field_data(fields_data)
    manager.generate_ros2_field_config(fields_data)
    
    # Generate coverage path if requested
    if args.field_id:
        print(f"Generating coverage path for {args.field_id}...")
        waypoints = manager.generate_coverage_path(fields_data, args.field_id, args.swath_width)
        
        if waypoints:
            # Save waypoints to file
            waypoints_file = f"coverage_path_{args.field_id}.json"
            with open(waypoints_file, 'w') as f:
                json.dump([{'lat': lat, 'lon': lon} for lat, lon in waypoints], f, indent=2)
            print(f"✅ Coverage path saved to {waypoints_file}")
    
    print("✅ Shapefile processing complete!")
    print(f"📁 Field data saved in: {manager.shapefiles_dir}")

if __name__ == '__main__':
    main()
