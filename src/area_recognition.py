#!/usr/bin/env python3
"""
area_recognition.py - Demonstrates the robot's ability to recognize 
familiar areas and adapt its navigation strategy accordingly.
"""

import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from mapping import Mapping
from navigation import Navigation

class AreaRecognitionSystem(Node):
    def __init__(self, map_file_path=None):
        super().__init__('area_recognition')
        
        # Initialize mapping and navigation systems
        self.mapping = Mapping(grid_size=0.2)  # 20cm grid cells
        self.navigation = Navigation(mapping_instance=self.mapping)
        
        # Map file location
        self.map_file = map_file_path if map_file_path else "saved_map.pkl"
        
        # Load existing map if available
        if self.map_file and os.path.exists(self.map_file):
            self.get_logger().info(f"Loading existing map from {self.map_file}")
            self.mapping.load_map(self.map_file)
            self.navigation.current_position = self.mapping.current_position
        else:
            self.get_logger().info("No existing map found. Starting with empty map.")

    def save_current_map(self):
        """Save the current map to file"""
        self.get_logger().info(f"Saving map to {self.map_file}")
        self.mapping.save_map(self.map_file)

    def process_lidar_data(self, lidar_data):
        """Process incoming LiDAR data for mapping and navigation"""
        # Update the map with new lidar data
        self.mapping.update_map(lidar_data, self.navigation.current_position)
        
        # Check if the area is familiar
        area_familiar = self.mapping.is_area_familiar()
        familiarity_level = self.mapping.get_current_area_familiarity()
        
        self.get_logger().info(f"Area familiarity level: {familiarity_level:.2f}")
        self.get_logger().info(f"Area is {'familiar' if area_familiar else 'not familiar'}")
        
        return area_familiar

    def navigate_to_goal(self, goal_position):
        """Navigate to a goal position with area awareness"""
        self.get_logger().info(f"Starting navigation to goal: {goal_position}")
        
        # Set the goal in the navigation system
        self.navigation.set_target(goal_position)
        
        # Plan a path considering area familiarity
        path = self.navigation.plan_path(
            self.navigation.current_position, 
            goal_position
        )
        
        self.get_logger().info(f"Planned path with {len(path)} waypoints")
        
        # Simulate navigation along the path
        for i, waypoint in enumerate(path[1:], 1):  # Skip the first point (current position)
            self.get_logger().info(f"Moving to waypoint {i}/{len(path)-1}: {waypoint}")
            
            # Simulate movement and processing of sensor data
            self.navigate_to_waypoint(waypoint)
            
            # Update position in the navigation system
            self.navigation.update_position(waypoint)
            
        self.get_logger().info(f"Reached goal position {goal_position}")
        return True

    def navigate_to_waypoint(self, waypoint):
        """Simulate navigation to a specific waypoint"""
        # Generate simulated LiDAR data for the current area
        lidar_data = self.simulate_lidar_sensing(waypoint)
        
        # Process the data to update mapping and check familiarity
        area_familiar = self.process_lidar_data(lidar_data)
        
        # Simulate movement behavior based on familiarity
        if area_familiar:
            self.get_logger().info("Using fast navigation in familiar area")
            # In a real system, you would set higher speed, simpler path
            time.sleep(0.5)  # Simulate faster movement
        else:
            self.get_logger().info("Using careful exploration in new area")
            # In a real system, you would use slower speed, more detailed sensing
            time.sleep(1.0)  # Simulate slower, more careful movement
            
            # In unfamiliar areas, we do more detailed mapping
            # This would typically involve additional sensor processing
            
        # Avoid obstacles along the way
        self.navigation.avoid_obstacles()
        
        # Mark this waypoint as visited
        self.mapping.remember_location(waypoint)
    
    def simulate_lidar_sensing(self, position):
        """Generate simulated LiDAR readings based on position"""
        # In a real system, this would come from actual sensors
        # For simulation, we'll create points in a circle around the position
        lidar_points = []
        radius = 2.0  # meters
        num_points = 16
        
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            distance = radius * (0.8 + 0.4 * np.random.random())  # Randomize a bit
            x = position[0] + distance * np.cos(angle)
            y = position[1] + distance * np.sin(angle)
            lidar_points.append((x, y))
        
        return lidar_points

def demo_area_recognition():
    """Demonstrate the area recognition system with a simulated mission"""
    # Initialize ROS
    rclpy.init()
    
    # Create the system
    system = AreaRecognitionSystem("demo_map.pkl")
    
    # Set initial position (0,0)
    start_position = (0.0, 0.0)
    system.navigation.update_position(start_position)
    
    try:
        # First navigation task - going to a new area
        system.get_logger().info("\n=== MISSION 1: EXPLORING NEW AREA ===")
        goal1 = (5.0, 5.0)
        system.navigate_to_goal(goal1)
        
        # Save the map after first exploration
        system.save_current_map()
        
        # Return to start - should now recognize the area
        system.get_logger().info("\n=== MISSION 2: RETURNING THROUGH FAMILIAR AREA ===")
        system.navigate_to_goal(start_position)
        
        # Go to another new area
        system.get_logger().info("\n=== MISSION 3: EXPLORING ANOTHER NEW AREA ===")
        goal2 = (-5.0, 3.0)
        system.navigate_to_goal(goal2)
        
        # Save the updated map
        system.save_current_map()
    
    except KeyboardInterrupt:
        system.get_logger().info("\nNavigation interrupted by user")
    finally:
        # Always save the map before exiting
        system.save_current_map()
        system.destroy_node()
        rclpy.shutdown()
        
    system.get_logger().info("Area recognition demonstration completed")

def main(args=None):
    rclpy.init(args=args)
    area_recognition = AreaRecognitionSystem()
    rclpy.spin(area_recognition)
    area_recognition.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    demo_area_recognition()