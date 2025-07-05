#!/usr/bin/env python3
"""
John Deere GPS Line Importer for Tractobots
Handles various John Deere GPS formats and guidance line data
"""

import os
import sys
import json
import xml.etree.ElementTree as ET
import csv
from pathlib import Path
from typing import List, Dict, Optional, Tuple
import logging
from datetime import datetime

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class JohnDeereGPSImporter:
    """
    Imports GPS guidance lines and field data from John Deere formats
    Supports: .xml (Operations Center), .csv (guidance lines), .kml, .gpx
    """
    
    def __init__(self, output_dir: str = "field_data/john_deere"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # John Deere specific formats
        self.supported_formats = [
            '.xml',   # Operations Center export
            '.csv',   # Guidance line export
            '.kml',   # Google Earth format
            '.gpx',   # GPS exchange format
            '.json',  # JSON export
            '.agx'    # Ag Leader/John Deere format
        ]
        
    def import_file(self, file_path: str) -> Dict:
        """
        Import GPS data from various John Deere formats
        
        Args:
            file_path: Path to the GPS data file
            
        Returns:
            Dictionary containing imported GPS data
        """
        return self.import_from_file(file_path)
    
    def import_from_file(self, file_path: str) -> Dict:
        """
        Import GPS data from various John Deere formats (alias for compatibility)
        
        Args:
            file_path: Path to the GPS data file
            
        Returns:
            Dictionary containing imported GPS data
        """
        file_path = Path(file_path)
        
        if not file_path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")
        
        file_ext = file_path.suffix.lower()
        
        if file_ext not in self.supported_formats:
            raise ValueError(f"Unsupported file format: {file_ext}")
        
        logger.info(f"Importing John Deere GPS data from {file_path}")
        
        if file_ext == '.xml':
            return self._import_xml(file_path)
        elif file_ext == '.csv':
            return self._import_csv(file_path)
        elif file_ext == '.kml':
            return self._import_kml(file_path)
        elif file_ext == '.gpx':
            return self._import_gpx(file_path)
        elif file_ext == '.json':
            return self._import_json(file_path)
        elif file_ext == '.agx':
            return self._import_agx(file_path)
        else:
            raise ValueError(f"Handler not implemented for {file_ext}")
    
    def _import_xml(self, file_path: Path) -> Dict:
        """Import from John Deere Operations Center XML export"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            gps_data = {
                'source': 'john_deere_xml',
                'filename': file_path.name,
                'imported_at': datetime.now().isoformat(),
                'guidance_lines': [],
                'field_boundaries': [],
                'waypoints': [],
                'metadata': {}
            }
            
            # Extract metadata
            gps_data['metadata'] = self._extract_xml_metadata(root)
            
            # Extract guidance lines
            guidance_lines = self._extract_guidance_lines_xml(root)
            gps_data['guidance_lines'].extend(guidance_lines)
            
            # Extract field boundaries
            boundaries = self._extract_field_boundaries_xml(root)
            gps_data['field_boundaries'].extend(boundaries)
            
            # Extract waypoints/track points
            waypoints = self._extract_waypoints_xml(root)
            gps_data['waypoints'].extend(waypoints)
            
            logger.info(f"Imported {len(guidance_lines)} guidance lines, {len(boundaries)} boundaries")
            return gps_data
            
        except Exception as e:
            logger.error(f"Error importing XML: {e}")
            raise
    
    def _import_csv(self, file_path: Path) -> Dict:
        """Import from CSV guidance line export"""
        try:
            gps_data = {
                'source': 'john_deere_csv',
                'filename': file_path.name,
                'imported_at': datetime.now().isoformat(),
                'guidance_lines': [],
                'waypoints': [],
                'metadata': {}
            }
            
            with open(file_path, 'r', newline='', encoding='utf-8') as csvfile:
                # Try to detect John Deere CSV format
                first_line = csvfile.readline()
                csvfile.seek(0)
                
                # Common John Deere CSV headers
                if 'latitude' in first_line.lower() and 'longitude' in first_line.lower():
                    reader = csv.DictReader(csvfile)
                    
                    current_line = []
                    line_id = 1
                    
                    for row in reader:
                        try:
                            # Extract coordinates
                            lat = float(row.get('Latitude', row.get('latitude', row.get('LAT', 0))))
                            lon = float(row.get('Longitude', row.get('longitude', row.get('LON', 0))))
                            
                            if lat != 0 and lon != 0:
                                point = {
                                    'lat': lat,
                                    'lon': lon,
                                    'elevation': float(row.get('Elevation', row.get('elevation', 0))),
                                    'heading': float(row.get('Heading', row.get('heading', 0))),
                                    'speed': float(row.get('Speed', row.get('speed', 0)))
                                }
                                
                                # Check if this is a new guidance line (based on distance or line_id)
                                line_id_field = row.get('LineID', row.get('line_id', row.get('Line', '')))
                                if line_id_field and line_id_field != str(line_id):
                                    # Save current line
                                    if current_line:
                                        gps_data['guidance_lines'].append({
                                            'line_id': f"line_{line_id}",
                                            'points': current_line.copy(),
                                            'type': 'ab_line'
                                        })
                                    
                                    current_line = []
                                    line_id = int(line_id_field) if line_id_field.isdigit() else line_id + 1
                                
                                current_line.append(point)
                                
                        except (ValueError, KeyError) as e:
                            logger.warning(f"Skipping invalid row: {e}")
                            continue
                    
                    # Save final line
                    if current_line:
                        gps_data['guidance_lines'].append({
                            'line_id': f"line_{line_id}",
                            'points': current_line,
                            'type': 'ab_line'
                        })
                
                else:
                    # Try alternative CSV format
                    csvfile.seek(0)
                    reader = csv.reader(csvfile)
                    
                    for i, row in enumerate(reader):
                        if len(row) >= 2:
                            try:
                                lat = float(row[0])
                                lon = float(row[1])
                                
                                gps_data['waypoints'].append({
                                    'lat': lat,
                                    'lon': lon,
                                    'point_id': f"point_{i}"
                                })
                            except ValueError:
                                continue
            
            logger.info(f"Imported {len(gps_data['guidance_lines'])} guidance lines from CSV")
            return gps_data
            
        except Exception as e:
            logger.error(f"Error importing CSV: {e}")
            raise
    
    def _import_kml(self, file_path: Path) -> Dict:
        """Import from KML/Google Earth export"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            # Handle KML namespace
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            
            gps_data = {
                'source': 'john_deere_kml',
                'filename': file_path.name,
                'imported_at': datetime.now().isoformat(),
                'guidance_lines': [],
                'field_boundaries': [],
                'waypoints': [],
                'metadata': {}
            }
            
            # Extract placemarks (guidance lines, boundaries, points)
            placemarks = root.findall('.//kml:Placemark', ns)
            
            for placemark in placemarks:
                name = placemark.find('kml:name', ns)
                name_text = name.text if name is not None else 'Unknown'
                
                # Check for LineString (guidance lines)
                linestring = placemark.find('.//kml:LineString/kml:coordinates', ns)
                if linestring is not None:
                    points = self._parse_kml_coordinates(linestring.text)
                    
                    gps_data['guidance_lines'].append({
                        'line_id': name_text,
                        'points': points,
                        'type': 'guidance_line'
                    })
                
                # Check for Polygon (field boundaries)
                polygon = placemark.find('.//kml:Polygon/kml:outerBoundaryIs/kml:LinearRing/kml:coordinates', ns)
                if polygon is not None:
                    points = self._parse_kml_coordinates(polygon.text)
                    
                    gps_data['field_boundaries'].append({
                        'boundary_id': name_text,
                        'points': points,
                        'type': 'field_boundary'
                    })
                
                # Check for Point (waypoints)
                point = placemark.find('.//kml:Point/kml:coordinates', ns)
                if point is not None:
                    coords = self._parse_kml_coordinates(point.text)
                    if coords:
                        gps_data['waypoints'].append({
                            'point_id': name_text,
                            'lat': coords[0]['lat'],
                            'lon': coords[0]['lon'],
                            'elevation': coords[0].get('elevation', 0)
                        })
            
            logger.info(f"Imported KML with {len(gps_data['guidance_lines'])} lines, {len(gps_data['field_boundaries'])} boundaries")
            return gps_data
            
        except Exception as e:
            logger.error(f"Error importing KML: {e}")
            raise
    
    def _import_gpx(self, file_path: Path) -> Dict:
        """Import from GPX format"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            # Handle GPX namespace
            ns = {'gpx': 'http://www.topografix.com/GPX/1/1'}
            
            gps_data = {
                'source': 'john_deere_gpx',
                'filename': file_path.name,
                'imported_at': datetime.now().isoformat(),
                'guidance_lines': [],
                'waypoints': [],
                'tracks': [],
                'metadata': {}
            }
            
            # Extract waypoints
            waypoints = root.findall('gpx:wpt', ns)
            for wpt in waypoints:
                lat = float(wpt.get('lat', 0))
                lon = float(wpt.get('lon', 0))
                name = wpt.find('gpx:name', ns)
                name_text = name.text if name is not None else f"waypoint_{len(gps_data['waypoints'])}"
                
                gps_data['waypoints'].append({
                    'point_id': name_text,
                    'lat': lat,
                    'lon': lon
                })
            
            # Extract tracks (guidance lines)
            tracks = root.findall('gpx:trk', ns)
            for track in tracks:
                track_name = track.find('gpx:name', ns)
                track_name_text = track_name.text if track_name is not None else f"track_{len(gps_data['guidance_lines'])}"
                
                track_points = []
                track_segments = track.findall('gpx:trkseg', ns)
                
                for segment in track_segments:
                    points = segment.findall('gpx:trkpt', ns)
                    for point in points:
                        lat = float(point.get('lat', 0))
                        lon = float(point.get('lon', 0))
                        elevation = point.find('gpx:ele', ns)
                        ele_value = float(elevation.text) if elevation is not None else 0
                        
                        track_points.append({
                            'lat': lat,
                            'lon': lon,
                            'elevation': ele_value
                        })
                
                if track_points:
                    gps_data['guidance_lines'].append({
                        'line_id': track_name_text,
                        'points': track_points,
                        'type': 'track'
                    })
            
            logger.info(f"Imported GPX with {len(gps_data['guidance_lines'])} tracks, {len(gps_data['waypoints'])} waypoints")
            return gps_data
            
        except Exception as e:
            logger.error(f"Error importing GPX: {e}")
            raise
    
    def _import_json(self, file_path: Path) -> Dict:
        """Import from JSON export"""
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            # Standardize the format
            gps_data = {
                'source': 'john_deere_json',
                'filename': file_path.name,
                'imported_at': datetime.now().isoformat(),
                'guidance_lines': data.get('guidance_lines', []),
                'field_boundaries': data.get('field_boundaries', []),
                'waypoints': data.get('waypoints', []),
                'metadata': data.get('metadata', {})
            }
            
            return gps_data
            
        except Exception as e:
            logger.error(f"Error importing JSON: {e}")
            raise
    
    def _import_agx(self, file_path: Path) -> Dict:
        """Import from AGX format (binary format - limited support)"""
        logger.warning("AGX format support is limited. Consider converting to XML/CSV first.")
        
        # AGX is a binary format - would need specialized library
        # For now, return empty structure
        return {
            'source': 'john_deere_agx',
            'filename': file_path.name,
            'imported_at': datetime.now().isoformat(),
            'guidance_lines': [],
            'field_boundaries': [],
            'waypoints': [],
            'metadata': {'note': 'AGX format requires specialized conversion tool'}
        }
    
    def _extract_xml_metadata(self, root) -> Dict:
        """Extract metadata from XML"""
        metadata = {}
        
        # Try to find common John Deere XML elements
        for elem in root.iter():
            if elem.tag.lower() in ['farm', 'field', 'client', 'grower']:
                if elem.text:
                    metadata[elem.tag.lower()] = elem.text
            
            # Extract attributes
            for attr, value in elem.attrib.items():
                if attr.lower() in ['name', 'id', 'date', 'version']:
                    metadata[f"{elem.tag}_{attr}"] = value
        
        return metadata
    
    def _extract_guidance_lines_xml(self, root) -> List[Dict]:
        """Extract guidance lines from XML"""
        guidance_lines = []
        
        # Look for common guidance line patterns in John Deere XML
        for elem in root.iter():
            if 'guidance' in elem.tag.lower() or 'line' in elem.tag.lower():
                points = []
                
                # Look for coordinate elements
                for point_elem in elem.iter():
                    if 'point' in point_elem.tag.lower() or 'coordinate' in point_elem.tag.lower():
                        lat = lon = None
                        
                        for coord in point_elem.iter():
                            if coord.tag.lower() in ['lat', 'latitude']:
                                lat = float(coord.text) if coord.text else None
                            elif coord.tag.lower() in ['lon', 'longitude', 'lng']:
                                lon = float(coord.text) if coord.text else None
                        
                        if lat is not None and lon is not None:
                            points.append({'lat': lat, 'lon': lon})
                
                if points:
                    guidance_lines.append({
                        'line_id': elem.get('id', f"line_{len(guidance_lines)}"),
                        'points': points,
                        'type': 'guidance_line'
                    })
        
        return guidance_lines
    
    def _extract_field_boundaries_xml(self, root) -> List[Dict]:
        """Extract field boundaries from XML"""
        boundaries = []
        
        for elem in root.iter():
            if 'boundary' in elem.tag.lower() or 'field' in elem.tag.lower():
                points = []
                
                for point_elem in elem.iter():
                    if 'point' in point_elem.tag.lower():
                        lat = lon = None
                        
                        for coord in point_elem.iter():
                            if coord.tag.lower() in ['lat', 'latitude']:
                                lat = float(coord.text) if coord.text else None
                            elif coord.tag.lower() in ['lon', 'longitude']:
                                lon = float(coord.text) if coord.text else None
                        
                        if lat is not None and lon is not None:
                            points.append({'lat': lat, 'lon': lon})
                
                if points:
                    boundaries.append({
                        'boundary_id': elem.get('id', f"boundary_{len(boundaries)}"),
                        'points': points,
                        'type': 'field_boundary'
                    })
        
        return boundaries
    
    def _extract_waypoints_xml(self, root) -> List[Dict]:
        """Extract waypoints from XML"""
        waypoints = []
        
        for elem in root.iter():
            if 'waypoint' in elem.tag.lower() or 'point' in elem.tag.lower():
                lat = lon = None
                
                # Try direct attributes first
                lat = elem.get('lat') or elem.get('latitude')
                lon = elem.get('lon') or elem.get('longitude')
                
                # Try child elements
                if not lat or not lon:
                    for child in elem:
                        if child.tag.lower() in ['lat', 'latitude']:
                            lat = child.text
                        elif child.tag.lower() in ['lon', 'longitude']:
                            lon = child.text
                
                if lat and lon:
                    try:
                        waypoints.append({
                            'point_id': elem.get('id', f"point_{len(waypoints)}"),
                            'lat': float(lat),
                            'lon': float(lon)
                        })
                    except ValueError:
                        continue
        
        return waypoints
    
    def _parse_kml_coordinates(self, coord_text: str) -> List[Dict]:
        """Parse KML coordinate string"""
        points = []
        
        if not coord_text:
            return points
        
        # KML coordinates are in lon,lat,elevation format
        coord_text = coord_text.strip()
        coord_pairs = coord_text.split()
        
        for pair in coord_pairs:
            parts = pair.split(',')
            if len(parts) >= 2:
                try:
                    lon = float(parts[0])
                    lat = float(parts[1])
                    elevation = float(parts[2]) if len(parts) > 2 else 0
                    
                    points.append({
                        'lat': lat,
                        'lon': lon,
                        'elevation': elevation
                    })
                except ValueError:
                    continue
        
        return points
    
    def export_to_tractobots_format(self, gps_data: Dict, output_path: str = None) -> str:
        """
        Export John Deere GPS data to Tractobots compatible format
        
        Args:
            gps_data: Imported GPS data
            output_path: Optional output file path
            
        Returns:
            Path to exported file
        """
        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_path = self.output_dir / f"tractobots_gps_{timestamp}.json"
        
        # Convert to Tractobots format
        tractobots_data = {
            'metadata': {
                'source': gps_data.get('source', 'john_deere'),
                'original_file': gps_data.get('filename', 'unknown'),
                'imported_at': gps_data.get('imported_at'),
                'converted_at': datetime.now().isoformat(),
                'format_version': '1.0'
            },
            'fields': [],
            'guidance_lines': gps_data.get('guidance_lines', []),
            'waypoints': gps_data.get('waypoints', [])
        }
        
        # Convert field boundaries to field format
        for boundary in gps_data.get('field_boundaries', []):
            field = {
                'field_id': boundary.get('boundary_id', 'unknown'),
                'field_name': boundary.get('boundary_id', 'Unknown Field'),
                'boundary_points': boundary.get('points', []),
                'attributes': {
                    'source': 'john_deere',
                    'type': 'imported_boundary'
                }
            }
            tractobots_data['fields'].append(field)
        
        # Save to file
        with open(output_path, 'w') as f:
            json.dump(tractobots_data, f, indent=2)
        
        logger.info(f"Exported Tractobots format to {output_path}")
        return str(output_path)
    
    def create_guidance_lines_for_ros2(self, gps_data: Dict, output_path: str = None) -> str:
        """
        Create ROS2 compatible guidance line data
        
        Args:
            gps_data: Imported GPS data
            output_path: Optional output file path
            
        Returns:
            Path to ROS2 format file
        """
        try:
            import yaml
        except ImportError:
            # Fallback to JSON if yaml not available
            yaml = None
        
        if output_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            ext = '.yaml' if yaml else '.json'
            output_path = self.output_dir / f"ros2_guidance_lines_{timestamp}{ext}"
        
        # Create ROS2 compatible format
        ros2_data = {
            'guidance_lines': {
                'header': {
                    'frame_id': 'map',
                    'stamp': datetime.now().isoformat()
                },
                'lines': []
            }
        }
        
        for line in gps_data.get('guidance_lines', []):
            ros2_line = {
                'id': line.get('line_id', 'unknown'),
                'type': line.get('type', 'ab_line'),
                'points': []
            }
            
            for point in line.get('points', []):
                ros2_point = {
                    'position': {
                        'x': point.get('lon', 0),  # Note: ROS typically uses x=east, y=north
                        'y': point.get('lat', 0),
                        'z': point.get('elevation', 0)
                    }
                }
                ros2_line['points'].append(ros2_point)
            
            ros2_data['guidance_lines']['lines'].append(ros2_line)
        
        # Save to file
        with open(output_path, 'w') as f:
            if yaml:
                yaml.dump(ros2_data, f, default_flow_style=False)
            else:
                json.dump(ros2_data, f, indent=2)
        
        logger.info(f"Created ROS2 guidance lines at {output_path}")
        return str(output_path)
    
    def export_to_gazebo_waypoints(self, gps_data: Dict, output_file: str = "jd_waypoints.yaml") -> str:
        """
        Export John Deere GPS lines to Gazebo-compatible waypoint format
        
        Args:
            gps_data: GPS data dictionary from import_file
            output_file: Path to output YAML file
            
        Returns:
            Path to created waypoints file
        """
        if not gps_data or 'guidance_lines' not in gps_data:
            raise ValueError("No GPS guidance lines available for export")
            
        logger.info(f"Exporting John Deere GPS lines to Gazebo waypoints: {output_file}")
        
        waypoints_data = {
            'waypoints': [],
            'field_info': {
                'name': gps_data.get('field_name', 'john_deere_field'),
                'generated_at': datetime.now().isoformat(),
                'coordinate_system': 'gps',
                'total_lines': len(gps_data['guidance_lines']),
                'total_waypoints': 0
            }
        }
        
        waypoint_id = 0
        for line_idx, line in enumerate(gps_data['guidance_lines']):
            if 'points' not in line:
                continue
                
            for point in line['points']:
                if 'lat' in point and 'lon' in point:
                    waypoint_data = {
                        'id': waypoint_id,
                        'line_id': line_idx,
                        'position': {
                            'x': self._convert_gps_to_meters(point['lon']),
                            'y': self._convert_gps_to_meters(point['lat']),
                            'z': 0.0
                        },
                        'orientation': {
                            'x': 0.0,
                            'y': 0.0,
                            'z': 0.0,
                            'w': 1.0
                        },
                        'speed': line.get('speed', 2.0),
                        'action': 'move',
                        'gps_coords': {
                            'lat': point['lat'],
                            'lon': point['lon']
                        }
                    }
                    waypoints_data['waypoints'].append(waypoint_data)
                    waypoint_id += 1
        
        waypoints_data['field_info']['total_waypoints'] = waypoint_id
        
        # Write to file
        output_path = Path(output_file)
        try:
            import yaml
            with open(output_path, 'w') as f:
                yaml.dump(waypoints_data, f, default_flow_style=False)
        except ImportError:
            # Fallback to JSON if PyYAML not available
            import json
            output_path = output_path.with_suffix('.json')
            with open(output_path, 'w') as f:
                json.dump(waypoints_data, f, indent=2)
            
        logger.info(f"Gazebo waypoints file created: {output_path}")
        return str(output_path)
    
    def export_to_gazebo_world(self, gps_data: Dict, output_file: str = "jd_field_world.sdf") -> str:
        """
        Export John Deere GPS field to Gazebo world format
        
        Args:
            gps_data: GPS data dictionary from import_file
            output_file: Path to output SDF world file
            
        Returns:
            Path to created world file
        """
        if not gps_data:
            raise ValueError("No GPS data available for export")
            
        logger.info(f"Exporting John Deere GPS field to Gazebo world: {output_file}")
        
        # Calculate field bounds from GPS data
        all_points = []
        if 'guidance_lines' in gps_data:
            for line in gps_data['guidance_lines']:
                if 'points' in line:
                    for point in line['points']:
                        if 'lat' in point and 'lon' in point:
                            all_points.append([point['lon'], point['lat']])
        
        if not all_points:
            raise ValueError("No GPS points available for field creation")
        
        # Convert to numpy array for easier calculation
        import numpy as np
        points_array = np.array(all_points)
        min_x, min_y = points_array.min(axis=0)
        max_x, max_y = points_array.max(axis=0)
        
        # Field dimensions (convert GPS to meters)
        field_width = self._convert_gps_to_meters(max_x - min_x)
        field_height = self._convert_gps_to_meters(max_y - min_y)
        
        # Ensure minimum size
        field_width = max(field_width, 100)
        field_height = max(field_height, 100)
        
        # Generate SDF content
        sdf_content = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="john_deere_field">
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Field base -->
    <model name="field_base">
      <pose>0 0 0.01 0 0 0</pose>
      <static>true</static>
      <link name="field_link">
        <collision name="field_collision">
          <geometry>
            <box>
              <size>{field_width} {field_height} 0.02</size>
            </box>
          </geometry>
        </collision>
        <visual name="field_visual">
          <geometry>
            <box>
              <size>{field_width} {field_height} 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.7 0.3 1</ambient>
            <diffuse>0.4 0.7 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- GPS guidance lines -->
'''
        
        # Add guidance lines as visual markers
        if 'guidance_lines' in gps_data:
            for line_idx, line in enumerate(gps_data['guidance_lines']):
                if 'points' not in line or len(line['points']) < 2:
                    continue
                
                points = line['points']
                for i in range(len(points) - 1):
                    start_point = points[i]
                    end_point = points[i + 1]
                    
                    if 'lat' not in start_point or 'lon' not in start_point:
                        continue
                    if 'lat' not in end_point or 'lon' not in end_point:
                        continue
                    
                    # Convert GPS to meters
                    start_x = self._convert_gps_to_meters(start_point['lon'])
                    start_y = self._convert_gps_to_meters(start_point['lat'])
                    end_x = self._convert_gps_to_meters(end_point['lon'])
                    end_y = self._convert_gps_to_meters(end_point['lat'])
                    
                    # Calculate line position and orientation
                    mid_x = (start_x + end_x) / 2
                    mid_y = (start_y + end_y) / 2
                    
                    # Calculate line length and angle
                    dx = end_x - start_x
                    dy = end_y - start_y
                    length = np.sqrt(dx**2 + dy**2)
                    angle = np.arctan2(dy, dx)
                    
                    sdf_content += f'''
    <!-- Guidance line {line_idx} segment {i} -->
    <model name="guidance_line_{line_idx}_{i}">
      <pose>{mid_x} {mid_y} 0.1 0 0 {angle}</pose>
      <static>true</static>
      <link name="guidance_link">
        <collision name="guidance_collision">
          <geometry>
            <box>
              <size>{length} 0.05 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="guidance_visual">
          <geometry>
            <box>
              <size>{length} 0.05 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
        
        # Add waypoint markers
        waypoint_idx = 0
        if 'guidance_lines' in gps_data:
            for line in gps_data['guidance_lines']:
                if 'points' in line:
                    for point in line['points']:
                        if 'lat' in point and 'lon' in point:
                            x = self._convert_gps_to_meters(point['lon'])
                            y = self._convert_gps_to_meters(point['lat'])
                            
                            sdf_content += f'''
    <!-- Waypoint {waypoint_idx} -->
    <model name="waypoint_{waypoint_idx}">
      <pose>{x} {y} 0.3 0 0 0</pose>
      <static>true</static>
      <link name="waypoint_link">
        <collision name="waypoint_collision">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="waypoint_visual">
          <geometry>
            <sphere>
              <radius>0.3</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
                            waypoint_idx += 1
        
        # Close the world
        sdf_content += '''
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <!-- Scene settings -->
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.6 0.8 1.0</background>
      <shadows>true</shadows>
    </scene>
    
    <!-- Wind simulation -->
    <wind>
      <linear_velocity>2 0 0</linear_velocity>
    </wind>
    
  </world>
</sdf>
'''
        
        # Write to file
        output_path = Path(output_file)
        with open(output_path, 'w') as f:
            f.write(sdf_content)
            
        logger.info(f"Gazebo world file created: {output_path}")
        return str(output_path)
    
    def _convert_gps_to_meters(self, gps_value: float) -> float:
        """
        Convert GPS coordinates to meters (simplified conversion)
        
        Args:
            gps_value: GPS coordinate value
            
        Returns:
            Value in meters
        """
        # Simple conversion for simulation purposes
        # In real applications, use proper coordinate transformation
        return gps_value * 111320  # Approximate meters per degree
    
    # ...existing code...
def main():
    """Command line interface for John Deere GPS importer"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Import John Deere GPS data")
    parser.add_argument('file', help="Path to John Deere GPS file")
    parser.add_argument('--output-dir', default='field_data/john_deere', 
                       help="Output directory for converted files")
    parser.add_argument('--format', choices=['tractobots', 'ros2', 'both'], 
                       default='both', help="Output format")
    parser.add_argument('--verbose', '-v', action='store_true', 
                       help="Verbose output")
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Import GPS data
    importer = JohnDeereGPSImporter(args.output_dir)
    
    try:
        print(f"🚜 Importing John Deere GPS data from {args.file}")
        gps_data = importer.import_file(args.file)
        
        print(f"✅ Successfully imported:")
        print(f"   - {len(gps_data.get('guidance_lines', []))} guidance lines")
        print(f"   - {len(gps_data.get('field_boundaries', []))} field boundaries")
        print(f"   - {len(gps_data.get('waypoints', []))} waypoints")
        
        # Export in requested formats
        if args.format in ['tractobots', 'both']:
            tractobots_file = importer.export_to_tractobots_format(gps_data)
            print(f"📄 Tractobots format: {tractobots_file}")
        
        if args.format in ['ros2', 'both']:
            ros2_file = importer.create_guidance_lines_for_ros2(gps_data)
            print(f"🤖 ROS2 format: {ros2_file}")
        
        print("\n🎉 Import completed successfully!")
        print(f"📁 Files saved to: {args.output_dir}")
        
    except Exception as e:
        print(f"❌ Error importing GPS data: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
