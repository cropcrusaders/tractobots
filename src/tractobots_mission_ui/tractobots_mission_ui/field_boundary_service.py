#!/usr/bin/env python3
"""
Field Boundary Service Node for Tractobots
ROS2 service to provide field boundary and path planning from shapefiles
"""

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped, PoseArray, Pose
from nav_msgs.msg import Path
from geographic_msgs.msg import GeoPoint, GeoPoseStamped

import json
import yaml
from pathlib import Path
from typing import List, Dict, Optional
import numpy as np

# Import our shapefile manager
try:
    from .shapefile_manager import ShapefileFieldManager
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(__file__))
    from shapefile_manager import ShapefileFieldManager

class FieldBoundaryService(Node):
    """
    ROS2 service node for field boundary management
    """
    
    def __init__(self):
        super().__init__('field_boundary_service')
        
        # Initialize shapefile manager
        self.shapefile_manager = ShapefileFieldManager()
        
        # Service for loading field boundaries
        self.load_field_service = self.create_service(
            String, 'load_field_boundaries', self.load_field_boundaries_callback)
        
        # Service for getting field list
        self.get_fields_service = self.create_service(
            String, 'get_field_list', self.get_field_list_callback)
        
        # Publishers
        self.field_boundary_pub = self.create_publisher(
            PoseArray, 'field_boundary', 10)
        
        self.coverage_path_pub = self.create_publisher(
            Path, 'coverage_path', 10)
        
        self.field_info_pub = self.create_publisher(
            String, 'field_info', 10)
        
        # Subscribers
        self.shapefile_sub = self.create_subscription(
            String, 'load_shapefile', self.load_shapefile_callback, 10)
        
        # Current field data
        self.current_fields_data = None
        self.active_field_id = None
        
        # Parameters
        self.declare_parameter('default_swath_width', 5.0)
        self.declare_parameter('field_data_dir', 'field_data')
        self.declare_parameter('auto_load_latest', True)
        
        self.get_logger().info("Field Boundary Service initialized")
        
        # Auto-load if enabled
        if self.get_parameter('auto_load_latest').value:
            self.auto_load_latest_field_data()
    
    def auto_load_latest_field_data(self):
        """Automatically load the most recent field data"""
        try:
            field_data_dir = Path(self.get_parameter('field_data_dir').value)
            json_files = list(field_data_dir.glob('field_boundaries*.json'))
            
            if json_files:
                # Load most recent file
                latest_file = max(json_files, key=lambda p: p.stat().st_mtime)
                self.get_logger().info(f"Auto-loading field data from {latest_file}")
                
                with open(latest_file, 'r') as f:
                    self.current_fields_data = json.load(f)
                
                self.get_logger().info(f"Loaded {len(self.current_fields_data['fields'])} fields")
                self.publish_field_info()
            
        except Exception as e:
            self.get_logger().warn(f"Could not auto-load field data: {e}")
    
    def load_field_boundaries_callback(self, request, response):
        """Service callback to load field boundaries from shapefile"""
        try:
            shapefile_path = request.data
            self.get_logger().info(f"Loading field boundaries from {shapefile_path}")
            
            # Load shapefile
            fields_data = self.shapefile_manager.load_field_boundaries(shapefile_path)
            
            if fields_data:
                self.current_fields_data = fields_data
                
                # Save processed data
                self.shapefile_manager.save_field_data(fields_data)
                self.shapefile_manager.generate_ros2_field_config(fields_data)
                
                response.data = f"Successfully loaded {len(fields_data['fields'])} fields"
                self.get_logger().info(response.data)
                
                # Publish field info
                self.publish_field_info()
                
            else:
                response.data = "Failed to load shapefile"
                self.get_logger().error(response.data)
            
        except Exception as e:
            response.data = f"Error: {str(e)}"
            self.get_logger().error(response.data)
        
        return response
    
    def get_field_list_callback(self, request, response):
        """Service callback to get list of available fields"""
        try:
            if not self.current_fields_data:
                response.data = "No field data loaded"
                return response
            
            field_list = []
            for field in self.current_fields_data['fields']:
                field_info = {
                    'id': field['field_id'],
                    'name': field['attributes'].get('NAME', field['field_id']),
                    'area_hectares': field.get('area_hectares', 0.0),
                    'center': field['center']
                }
                field_list.append(field_info)
            
            response.data = json.dumps(field_list, indent=2)
            
        except Exception as e:
            response.data = f"Error: {str(e)}"
            self.get_logger().error(response.data)
        
        return response
    
    def load_shapefile_callback(self, msg):
        """Subscriber callback to load shapefile from topic"""
        shapefile_path = msg.data
        self.get_logger().info(f"Received shapefile load request: {shapefile_path}")
        
        # Load shapefile
        fields_data = self.shapefile_manager.load_field_boundaries(shapefile_path)
        
        if fields_data:
            self.current_fields_data = fields_data
            self.publish_field_info()
            self.get_logger().info(f"Loaded {len(fields_data['fields'])} fields from topic")
    
    def publish_field_info(self):
        """Publish current field information"""
        if not self.current_fields_data:
            return
        
        field_info = {
            'metadata': self.current_fields_data['metadata'],
            'num_fields': len(self.current_fields_data['fields']),
            'fields': []
        }
        
        for field in self.current_fields_data['fields']:
            field_summary = {
                'id': field['field_id'],
                'name': field['attributes'].get('NAME', field['field_id']),
                'area_hectares': field.get('area_hectares', 0.0),
                'center': field['center'],
                'num_boundary_points': len(field.get('boundary_points', []))
            }
            field_info['fields'].append(field_summary)
        
        msg = String()
        msg.data = json.dumps(field_info, indent=2)
        self.field_info_pub.publish(msg)
    
    def publish_field_boundary(self, field_id: str):
        """Publish field boundary as PoseArray"""
        if not self.current_fields_data:
            self.get_logger().error("No field data loaded")
            return
        
        # Find field
        field_data = None
        for field in self.current_fields_data['fields']:
            if field['field_id'] == field_id:
                field_data = field
                break
        
        if not field_data:
            self.get_logger().error(f"Field {field_id} not found")
            return
        
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        # Get boundary points
        boundary_points = field_data['boundary_points']
        if isinstance(boundary_points[0], list):
            # MultiPolygon - use first polygon
            boundary_points = boundary_points[0]
        
        for point in boundary_points:
            pose = Pose()
            pose.position.x = point['lon']  # Note: might need coordinate transformation
            pose.position.y = point['lat']
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.field_boundary_pub.publish(pose_array)
        self.get_logger().info(f"Published boundary for field {field_id} with {len(pose_array.poses)} points")
    
    def generate_and_publish_coverage_path(self, field_id: str, swath_width: Optional[float] = None):
        """Generate and publish coverage path for field"""
        if not self.current_fields_data:
            self.get_logger().error("No field data loaded")
            return
        
        if swath_width is None:
            swath_width = self.get_parameter('default_swath_width').value
        
        # Generate coverage path
        waypoints = self.shapefile_manager.generate_coverage_path(
            self.current_fields_data, field_id, swath_width)
        
        if not waypoints:
            self.get_logger().error(f"Failed to generate coverage path for field {field_id}")
            return
        
        # Create Path message
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        
        for i, (lat, lon) in enumerate(waypoints):
            pose_stamped = GeoPoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.latitude = lat
            pose_stamped.pose.position.longitude = lon
            pose_stamped.pose.position.altitude = 0.0
            pose_stamped.pose.orientation.w = 1.0
            
            # Convert to regular PoseStamped for Path message
            # Note: This is a simplified conversion - you may need proper coordinate transformation
            from geometry_msgs.msg import PoseStamped
            regular_pose = PoseStamped()
            regular_pose.header = pose_stamped.header
            regular_pose.pose.position.x = lon  # Simplified - needs proper transformation
            regular_pose.pose.position.y = lat
            regular_pose.pose.position.z = 0.0
            regular_pose.pose.orientation.w = 1.0
            
            path.poses.append(regular_pose)
        
        self.coverage_path_pub.publish(path)
        self.get_logger().info(f"Published coverage path for field {field_id} with {len(path.poses)} waypoints")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = FieldBoundaryService()
        
        # Example usage
        node.get_logger().info("Field Boundary Service ready!")
        node.get_logger().info("Available services:")
        node.get_logger().info("  - /load_field_boundaries (String)")
        node.get_logger().info("  - /get_field_list (String)")
        node.get_logger().info("Available topics:")
        node.get_logger().info("  - /load_shapefile (String) - input")
        node.get_logger().info("  - /field_boundary (PoseArray) - output")
        node.get_logger().info("  - /coverage_path (Path) - output")
        node.get_logger().info("  - /field_info (String) - output")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
