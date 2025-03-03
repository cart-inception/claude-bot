#!/usr/bin/env python3
import numpy as np
import os
import json
import pickle
from datetime import datetime
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class Mapping(Node):
    def __init__(self, grid_size=0.1, confidence_threshold=0.7):
        super().__init__('mapping')
        
        # Initialize map_data as a dictionary with a list to hold points
        self.map_data = {'points': []}
        self.visited_locations = set()
        
        # Grid-based area recognition parameters
        self.grid_size = grid_size  # Size of each grid cell in meters
        self.confidence_threshold = confidence_threshold  # Threshold to consider an area as "known"
        self.grid_map = {}  # Dictionary to store visited grid cells and their confidence values
        self.current_position = (0, 0)  # Current position (x, y) in meters
        
        # Map versioning and metadata
        self.map_created = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.map_last_updated = self.map_created
        
        # ROS 2 publishers
        self.map_publisher = self.create_publisher(OccupancyGrid, '/wave_rover/grid_map', 10)
        
        # Timer for periodic map updates
        self.map_update_timer = self.create_timer(5.0, self.publish_map)
    
    def create_map(self, lidar_data):
        try:
            # Process LiDAR data to create/update the map by appending new points
            if 'points' not in self.map_data:
                self.map_data['points'] = []
            self.map_data['points'].extend(lidar_data)
            
            # Update grid cells based on LiDAR data
            self._update_grid_from_lidar(lidar_data)
            return True
        except Exception as e:
            self.get_logger().error(f"Error creating map: {str(e)}")
            return False
    
    def update_map(self, new_data, robot_position=None):
        try:
            # Update the existing map with new data by adding only unique points
            if 'points' not in self.map_data:
                self.map_data['points'] = []
            
            if robot_position:
                self.current_position = robot_position
            
            for point in new_data:
                if point not in self.map_data['points']:
                    self.map_data['points'].append(point)
            
            # Update grid cells based on new data
            self._update_grid_from_lidar(new_data)
            self.map_last_updated = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            return True
        except Exception as e:
            self.get_logger().error(f"Error updating map: {str(e)}")
            return False
    
    def _update_grid_from_lidar(self, lidar_points):
        """Update grid cells with confidence values based on LiDAR data"""
        try:
            for point in lidar_points:
                # Convert absolute point to grid cell
                grid_x = int(point[0] / self.grid_size)
                grid_y = int(point[1] / self.grid_size)
                grid_cell = (grid_x, grid_y)
                
                # Update confidence for this cell
                if grid_cell in self.grid_map:
                    # Increase confidence (saturate at 1.0)
                    self.grid_map[grid_cell] = min(1.0, self.grid_map[grid_cell] + 0.1)
                else:
                    # Initialize with moderate confidence
                    self.grid_map[grid_cell] = 0.5
            return True
        except Exception as e:
            self.get_logger().error(f"Error updating grid from LiDAR: {str(e)}")
            return False
    
    def get_current_area_familiarity(self, radius=5):
        """
        Determine how familiar the robot is with its current area.
        Returns a value between 0.0 (completely unknown) and 1.0 (very familiar)
        """
        try:
            if not self.current_position:
                return 0.0
                
            # Convert position to grid coordinates
            current_grid_x = int(self.current_position[0] / self.grid_size)
            current_grid_y = int(self.current_position[1] / self.grid_size)
            
            # Check cells in a square around the current position
            cells_checked = 0
            total_confidence = 0.0
            
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    grid_cell = (current_grid_x + dx, current_grid_y + dy)
                    if grid_cell in self.grid_map:
                        total_confidence += self.grid_map[grid_cell]
                    cells_checked += 1
            
            # Calculate average confidence of surrounding area
            if cells_checked > 0:
                return total_confidence / cells_checked
            return 0.0
        except Exception as e:
            self.get_logger().error(f"Error getting area familiarity: {str(e)}")
            return 0.0
    
    def is_area_familiar(self, position=None, threshold=None):
        """
        Check if the current area (or specified position) is familiar enough
        to skip detailed mapping and use navigation only
        """
        try:
            if position:
                current_pos = self.current_position
                self.current_position = position
                familiarity = self.get_current_area_familiarity()
                self.current_position = current_pos
            else:
                familiarity = self.get_current_area_familiarity()
            
            check_threshold = threshold if threshold is not None else self.confidence_threshold
            return familiarity >= check_threshold
        except Exception as e:
            self.get_logger().error(f"Error checking area familiarity: {str(e)}")
            return False
    
    def remember_location(self, location):
        """Remember a previously visited location"""
        try:
            # Remember a previously visited location
            self.visited_locations.add(tuple(location))
            
            # Also update the grid representation
            grid_x = int(location[0] / self.grid_size)
            grid_y = int(location[1] / self.grid_size)
            grid_cell = (grid_x, grid_y)
            
            # Mark the cell with high confidence
            self.grid_map[grid_cell] = 1.0
            return True
        except Exception as e:
            self.get_logger().error(f"Error remembering location: {str(e)}")
            return False
    
    def is_visited(self, location):
        """Check if a location has been visited"""
        try:
            return tuple(location) in self.visited_locations
        except Exception as e:
            self.get_logger().error(f"Error checking if location is visited: {str(e)}")
            return False
    
    def publish_map(self):
        """Publish the current map as an OccupancyGrid message"""
        try:
            if not self.grid_map:
                return  # No map data to publish
                
            # Create occupancy grid from our grid map
            grid = OccupancyGrid()
            grid.header = Header()
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.header.frame_id = "map"
            
            # Find grid boundaries
            min_x = min_y = float('inf')
            max_x = max_y = float('-inf')
            
            for grid_cell in self.grid_map.keys():
                x, y = grid_cell
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)
            
            # If there are no cells, just return
            if min_x == float('inf'):
                return
                
            # Add some padding
            padding = 10
            min_x -= padding
            min_y -= padding
            max_x += padding
            max_y += padding
            
            # Set dimensions
            width = max_x - min_x + 1
            height = max_y - min_y + 1
            
            grid.info.resolution = self.grid_size
            grid.info.width = int(width)
            grid.info.height = int(height)
            grid.info.origin.position.x = min_x * self.grid_size
            grid.info.origin.position.y = min_y * self.grid_size
            
            # Initialize the data array with -1 (unknown)
            data = [-1] * (int(width) * int(height))
            
            # Fill in the known cells
            for (x, y), confidence in self.grid_map.items():
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    # Convert grid coordinates to array index
                    array_x = x - min_x
                    array_y = y - min_y
                    index = int(array_y * width + array_x)
                    
                    if 0 <= index < len(data):
                        # Convert confidence to occupancy (0-100)
                        # Confidence 0.5 means unknown (50)
                        # Higher confidence means more likely to be occupied
                        occupancy = int(confidence * 100)
                        data[index] = occupancy
            
            grid.data = data
            self.map_publisher.publish(grid)
        except Exception as e:
            self.get_logger().error(f"Error publishing map: {str(e)}")
    
    def save_map(self, filename):
        """Save the current map with grid representation to a file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
            
            map_data = {
                'points': self.map_data['points'],
                'grid_map': self.grid_map,
                'grid_size': self.grid_size,
                'visited_locations': list(self.visited_locations),
                'created': self.map_created,
                'last_updated': self.map_last_updated,
                'current_position': self.current_position
            }
            
            # Save in pickle format for complete data preservation
            with open(filename, 'wb') as f:
                pickle.dump(map_data, f)
            
            # Also save a human-readable JSON version (without large point lists)
            json_data = {
                'grid_size': self.grid_size,
                'created': self.map_created,
                'last_updated': self.map_last_updated,
                'current_position': self.current_position,
                'grid_cells_count': len(self.grid_map),
                'points_count': len(self.map_data['points'])
            }
            
            json_filename = filename.rsplit('.', 1)[0] + '.json'
            with open(json_filename, 'w') as f:
                json.dump(json_data, f, indent=2)
                
            self.get_logger().info(f"Map saved to {filename}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error saving map: {str(e)}")
            return False
    
    def load_map(self, filename):
        """Load a map from a file"""
        try:
            with open(filename, 'rb') as f:
                data = pickle.load(f)
                
            self.map_data['points'] = data.get('points', [])
            self.grid_map = data.get('grid_map', {})
            self.grid_size = data.get('grid_size', 0.1)
            self.visited_locations = set(data.get('visited_locations', []))
            self.map_created = data.get('created', self.map_created)
            self.map_last_updated = data.get('last_updated', self.map_last_updated)
            self.current_position = data.get('current_position', (0, 0))
            
            # Publish the loaded map
            self.publish_map()
            self.get_logger().info(f"Map loaded from {filename}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {str(e)}")
            return False

def main(args=None):
    rclpy.init(args=args)
    mapping_node = Mapping()
    
    try:
        rclpy.spin(mapping_node)
    except KeyboardInterrupt:
        pass
    finally:
        mapping_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()