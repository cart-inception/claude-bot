#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import os
import pickle
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from utils.helper import log_message, process_point_cloud_data, save_map, load_map

class LidarIntegration(Node):
    def __init__(self):
        super().__init__('lidar_integration')
        
        self.lidar = None
        self.point_cloud_data = []
        self.scan_subscriber = None
        self.point_cloud_publisher = None
        self.map_data = None
        self.remembered_locations = []
        self.is_initialized = False
        
        # Map parameters
        self.map_resolution = 0.05  # 5cm per cell
        self.map_width = 1000      # 50m / 0.05 = 1000 cells
        self.map_height = 1000     # 50m / 0.05 = 1000 cells

    def initialize_lidar(self):
        """Initialize the LiDAR scanner and ROS connections"""
        try:
            # Subscribe to the laser scan topic
            self.scan_subscriber = self.create_subscription(
                LaserScan,
                '/scan',
                self.laser_scan_callback,
                10)
            
            # Create publisher for processed point cloud data
            self.point_cloud_publisher = self.create_publisher(
                PointCloud2,
                '/wave_rover/point_cloud',
                10)
            
            # Create publisher for remembered locations
            self.location_marker_publisher = self.create_publisher(
                MarkerArray,
                '/wave_rover/remembered_locations',
                10)
            
            # Create publisher for map data
            self.map_publisher = self.create_publisher(
                OccupancyGrid,
                '/wave_rover/map',
                10)
            
            log_message("LiDAR system initialized successfully")
            self.is_initialized = True
            return True
        except Exception as e:
            log_message(f"Error initializing LiDAR: {str(e)}")
            return False

    def laser_scan_callback(self, scan_data):
        """Callback function for laser scan data"""
        try:
            # Convert LaserScan to points
            points = []
            for i, distance in enumerate(scan_data.ranges):
                # Skip invalid measurements
                if distance < scan_data.range_min or distance > scan_data.range_max:
                    continue
                
                # Calculate angle and convert to Cartesian coordinates
                angle = scan_data.angle_min + i * scan_data.angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                z = 0.0  # 2D LiDAR, so Z is 0
                
                points.append({'x': x, 'y': y, 'z': z})
            
            # Process the point cloud data
            self.process_point_cloud(points)
            
        except Exception as e:
            log_message(f"Error in laser scan callback: {str(e)}")

    def process_point_cloud(self, data):
        """Process the incoming point cloud data"""
        try:
            # Use the helper function to process the point cloud data
            self.point_cloud_data = process_point_cloud_data(data)
            
            # Update the map with the new data
            self._update_map()
            
            # Publish the processed data
            self.publish_data()
            
        except Exception as e:
            log_message(f"Error processing point cloud: {str(e)}")

    def publish_data(self):
        """Publish the processed point cloud data to ROS topics"""
        if not self.is_initialized:
            log_message("LiDAR not initialized. Cannot publish data.")
            return
        
        try:
            # Create header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "lidar_frame"
            
            # Convert our point cloud data format to ROS PointCloud2 format
            points = []
            for point in self.point_cloud_data:
                points.append([point['x'], point['y'], point['z']])
            
            # Create PointCloud2 message
            pc2_msg = pc2.create_cloud_xyz32(header, points)
            
            # Publish point cloud
            self.point_cloud_publisher.publish(pc2_msg)
            
            # Publish map if available
            if self.map_data is not None:
                self._publish_map()
                
            # Publish remembered locations if any
            if self.remembered_locations:
                self._publish_locations()
                
        except Exception as e:
            log_message(f"Error publishing data: {str(e)}")

    def load_map(self, map_file):
        """Load a previously saved map"""
        try:
            if os.path.exists(map_file):
                self.map_data = load_map(map_file)
                log_message(f"Map loaded successfully from {map_file}")
                return True
            else:
                log_message(f"Map file {map_file} does not exist")
                return False
        except Exception as e:
            log_message(f"Error loading map: {str(e)}")
            return False

    def save_map(self, map_file):
        """Save the current map"""
        try:
            if self.map_data is not None:
                save_map(map_file, self.map_data)
                log_message(f"Map saved successfully to {map_file}")
                return True
            else:
                log_message("No map data to save")
                return False
        except Exception as e:
            log_message(f"Error saving map: {str(e)}")
            return False

    def remember_location(self, location):
        """Remember a previously visited location"""
        try:
            # Location should be a tuple or list with (x, y) coordinates
            if not isinstance(location, (list, tuple)) or len(location) < 2:
                log_message("Invalid location format. Expected (x, y)")
                return False
            
            # Convert to tuple for consistency
            loc = (float(location[0]), float(location[1]))
            
            # Check if the location is already remembered
            for existing_loc in self.remembered_locations:
                # If the location is close to an existing one, don't add it
                if np.sqrt((existing_loc[0] - loc[0])**2 + (existing_loc[1] - loc[1])**2) < 0.5:
                    log_message(f"Location {loc} is already remembered")
                    return False
            
            # Add the new location
            self.remembered_locations.append(loc)
            log_message(f"Remembered new location: {loc}")
            
            # Publish the updated locations
            self._publish_locations()
            return True
        except Exception as e:
            log_message(f"Error remembering location: {str(e)}")
            return False

    def get_map(self):
        """Return the current map"""
        return self.map_data

    def _update_map(self):
        """Update the internal map with new point cloud data"""
        # Initialize map if it doesn't exist
        if self.map_data is None:
            self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)
        
        # Update map based on point cloud data
        # This is a simple example - in a real application, you would use 
        # a proper SLAM or mapping algorithm
        for point in self.point_cloud_data:
            # Convert world coordinates to map coordinates
            map_x = int((point['x'] / self.map_resolution) + (self.map_width / 2))
            map_y = int((point['y'] / self.map_resolution) + (self.map_height / 2))
            
            # Check if the coordinates are within the map
            if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                # Mark as occupied (100 means 100% probability of obstacle)
                self.map_data[map_y, map_x] = 100
    
    def _publish_map(self):
        """Publish the map as an OccupancyGrid message"""
        if self.map_data is None:
            return
        
        # Create OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        
        # Set map metadata
        grid.info.resolution = self.map_resolution
        grid.info.width = self.map_width
        grid.info.height = self.map_height
        grid.info.origin.position.x = -self.map_width * self.map_resolution / 2
        grid.info.origin.position.y = -self.map_height * self.map_resolution / 2
        
        # Set map data (flattened)
        grid.data = self.map_data.flatten().tolist()
        
        # Publish the map
        self.map_publisher.publish(grid)
    
    def _publish_locations(self):
        """Publish remembered locations as markers"""
        if not self.remembered_locations:
            return
        
        marker_array = MarkerArray()
        for i, loc in enumerate(self.remembered_locations):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "remembered_locations"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = loc[0]
            marker.pose.position.y = loc[1]
            marker.pose.position.z = 0.5
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.location_marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    lidar_integration = LidarIntegration()
    lidar_integration.initialize_lidar()
    rclpy.spin(lidar_integration)
    lidar_integration.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()