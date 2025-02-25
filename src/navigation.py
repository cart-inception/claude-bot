import os
import json
import numpy as np
from utils.helper import log_message

class Navigation:
    def __init__(self, mapping_instance=None):
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
        # Load map from file (simple JSON based stub implementation)
        import os, json
        if os.path.exists(map_file):
            with open(map_file, 'r') as f:
                self.map_data = json.load(f)
            self.current_position = self.map_data.get('current_position', (0, 0))
        else:
            self.map_data = {}
    
    def save_map(self, map_file):
        # Save current map data (simple JSON based stub implementation)
        import json
        with open(map_file, 'w') as f:
            json.dump(self.map_data, f)

    def check_area_familiarity(self):
        """
        Check if the current area is familiar enough to switch to faster navigation mode
        """
        if self.mapping is None:
            return False
        
        is_familiar = self.mapping.is_area_familiar(self.current_position)
        
        # Set navigation mode based on familiarity
        if is_familiar and self.current_mode != self.MODE_FAMILIAR:
            self.current_mode = self.MODE_FAMILIAR
            log_message(f"Area recognized as familiar. Switching to {self.MODE_FAMILIAR} mode.")
            return True
        elif not is_familiar and self.current_mode != self.MODE_EXPLORING:
            self.current_mode = self.MODE_EXPLORING
            log_message(f"Area not familiar. Switching to {self.MODE_EXPLORING} mode.")
            return False
        
        return is_familiar
    
    def plan_path(self, start, goal):
        """
        Plan a path from start to goal, considering area familiarity
        """
        # Simple path planning with familiarity awareness
        self.path = [start]
        
        if self.mapping:
            # Check familiarity of start and goal areas
            start_familiar = self.mapping.is_area_familiar(start)
            goal_familiar = self.mapping.is_area_familiar(goal)
            
            # Different strategies based on familiarity
            if start_familiar and goal_familiar:
                # Both areas are familiar, can use direct path with fewer waypoints
                log_message("Planning direct path through familiar areas")
                self.path = [start, goal]
            else:
                # Need more cautious path with more waypoints for unfamiliar areas
                log_message("Planning cautious path through unfamiliar areas")
                # Simple intermediate waypoint (could be enhanced with actual path planning)
                midpoint = ((start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2)
                self.path = [start, midpoint, goal]
        else:
            # Default basic path without mapping information
            self.path = [start, goal]
        
        self.next_waypoint = self.path[1] if len(self.path) > 1 else goal
        return self.path
    
    def avoid_obstacles(self):
        # Adjust behavior based on familiarity mode
        if self.current_mode == self.MODE_FAMILIAR:
            log_message('Quick obstacle avoidance in familiar area')
        else:
            log_message('Careful obstacle avoidance and mapping in new area')
            
            # In exploring mode, remember obstacles we find
            if self.mapping:
                # This would come from sensor data in a real implementation
                obstacle_point = (self.current_position[0] + 0.5, self.current_position[1] + 0.5)
                self.mapping.update_map([obstacle_point], self.current_position)
    
    def move_to_target(self):
        """
        Move toward the target with behavior adapted to the current area familiarity
        """
        # First check area familiarity to adjust navigation strategy
        self.check_area_familiarity()
        
        if self.current_mode == self.MODE_FAMILIAR:
            log_message(f'Moving quickly to target {self.target_position} in familiar area')
            # We could use faster speed and simpler obstacle avoidance in familiar areas
        else:
            log_message(f'Exploring carefully toward target {self.target_position} in new area')
            # Slower movement with more detailed mapping in new areas
            
        # Actually move (simulation)
        log_message(f'Moving to target {self.target_position}')
        
        # If we have a mapping system, remember this location
        if self.mapping and self.current_position:
            self.mapping.remember_location(self.current_position)
    
    def update_position(self, new_position):
        # Update the current position of the robot
        self.current_position = new_position
        log_message(f'Updated current position to {new_position}')
        
        # Check if we should switch navigation modes based on area familiarity
        if self.mapping:
            self.check_area_familiarity()
    
    def set_target(self, target_position):
        # Set the target position for navigation
        self.target_position = target_position
        log_message(f'Set target position to {target_position}')
        
        # Plan a path if we have current position
        if self.current_position:
            self.plan_path(self.current_position, target_position)