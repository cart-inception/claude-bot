#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from utils.helper import log_message

class Navigation(Node):
    def __init__(self, mapping_instance=None):
        super().__init__('navigation')
        
        # Initialize navigation parameters
        self.current_position = None
        self.target_position = None
        self.map_data = None
        
        # Reference to the mapping system
        self.mapping = mapping_instance
        
        # Navigation modes
        self.MODE_EXPLORING = "exploring"  # Slow, cautious movement with mapping
        self.MODE_FAMILIAR = "familiar"    # Faster navigation through known area
        self.current_mode = self.MODE_EXPLORING
        
        # Path planning parameters
        self.path = []
        self.next_waypoint = None
    
    def set_mapping_reference(self, mapping_instance):
        """Set or update the reference to the mapping system"""
        self.mapping = mapping_instance
        
    def load_map(self, map_file):
        """Load a map from file"""
        if self.mapping:
            return self.mapping.load_map(map_file)
        return False
    
    def save_map(self, map_file):
        """Save the current map to file"""
        if self.mapping:
            return self.mapping.save_map(map_file)
        return False

    def check_area_familiarity(self):
        """Check if current area is familiar using mapping data"""
        if self.mapping:
            return self.mapping.is_area_familiar()
        return False
    
    def plan_path(self, start, goal):
        """Plan a path from start to goal"""
        # Simple placeholder implementation - in a real system this would use proper path planning
        if not start or not goal:
            return []
            
        # Create a simple straight-line path for demonstration
        path = [start]
        
        # Add a few intermediate points
        steps = max(3, int(((goal[0] - start[0])**2 + (goal[1] - start[1])**2)**0.5))
        for i in range(1, steps):
            t = i / steps
            x = start[0] + (goal[0] - start[0]) * t
            y = start[1] + (goal[1] - start[1]) * t
            path.append((x, y))
            
        path.append(goal)
        self.path = path
        return path
    
    def avoid_obstacles(self):
        """Implement obstacle avoidance behavior"""
        # Placeholder for obstacle avoidance logic
        if self.current_mode == self.MODE_EXPLORING:
            # More cautious obstacle avoidance in exploration mode
            log_message("Using cautious obstacle avoidance in exploration mode")
        else:
            # More efficient obstacle avoidance in familiar areas
            log_message("Using efficient obstacle avoidance in familiar area mode")
    
    def move_to_target(self):
        """Move the robot towards the target using the planned path"""
        if not self.path:
            log_message("No path available for navigation")
            return False
            
        if self.next_waypoint is None and len(self.path) > 0:
            self.next_waypoint = self.path[0]
            
        # Simulate movement to next waypoint
        if self.next_waypoint:
            log_message(f"Moving towards waypoint: {self.next_waypoint}")
            # In a real implementation, you would send movement commands to the robot here
            
            # Update waypoint
            if len(self.path) > 1:
                self.path.pop(0)
                self.next_waypoint = self.path[0]
            else:
                log_message("Reached final waypoint")
                self.next_waypoint = None
                self.path = []
            return True
        return False
    
    def update_position(self, new_position):
        """Update the robot's current position"""
        self.current_position = new_position
        log_message(f"Position updated to {new_position}")
    
    def set_target(self, target_position):
        """Set a new navigation target"""
        self.target_position = target_position
        log_message(f"New target set: {target_position}")
        
        # Reset path when target changes
        self.path = []
        self.next_waypoint = None

def main(args=None):
    rclpy.init(args=args)
    navigation_node = Navigation()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()