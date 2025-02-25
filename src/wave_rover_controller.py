#!/usr/bin/env python
import time
import random
import os
from mapping import Mapping
from navigation import Navigation
from lidar_integration import LidarIntegration
from area_recognition import AreaRecognitionSystem
from utils.helper import log_message, load_configuration
from gpio_controller import GpioController

class WaveRoverController:
    def __init__(self):
        # Load configuration from config.json located one directory up
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.json')
        self.config = load_configuration(config_path)
        
        # Initialize area recognition system which manages mapping and navigation
        map_file = self.config.get('map_file', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'saved_map.pkl'))
        self.area_recognition = AreaRecognitionSystem(map_file)
        
        # Keep references to mapping and navigation objects
        self.mapping = self.area_recognition.mapping
        self.navigation = self.area_recognition.navigation
        
        self.lidar = LidarIntegration()
        self.gpio_controller = None
        self.initialize_robot()
        self.running = False
        
        # Area familiarity tracking
        self.current_area_familiar = False
        self.familiar_areas_visited = 0
        self.new_areas_visited = 0

    def initialize_robot(self):
        log_message("Initializing robot hardware and software components")
        lidar_port = self.config.get('lidar_port', '/dev/ttyUSB0')
        self.lidar.initialize_lidar(lidar_port)
        
        # Initialize GPIO rover control - prioritizing GPIO control for Raspberry Pi
        left_pin = self.config.get('gpio_left_motor_pin')
        right_pin = self.config.get('gpio_right_motor_pin')
        
        if left_pin is not None and right_pin is not None:
            self.gpio_controller = GpioController(left_pin, right_pin)
            log_message(f"Initialized GPIO rover control on pins: {left_pin}, {right_pin}")
        else:
            # Fallback to serial control only if GPIO pins are not configured
            rover_port = self.config.get('rover_control_port', '/dev/ttyUSB1')
            log_message(f"GPIO pins not configured. Using rover control port: {rover_port}")
            # Additional code would be needed here to initialize serial communication
        
    def start_control_loop(self):
        self.running = True
        log_message("Starting control loop with area recognition enabled...")
        try:
            while self.running:
                self.handle_commands()
                time.sleep(1)  # simulate delay between iterations
                
                # Periodically save the map
                if random.random() < 0.1:  # ~10% chance each iteration
                    self.area_recognition.save_current_map()
                    log_message("Map saved automatically")
                    
        except KeyboardInterrupt:
            self.stop()

    def handle_commands(self):
        # Collect LiDAR data
        log_message("Collecting LiDAR data...")
        fake_data = [{"z": random.uniform(0, 2)} for _ in range(10)]
        self.lidar.process_point_cloud(fake_data)
        processed = self.lidar.point_cloud_data
        
        # Convert processed data to the format expected by our area recognition system
        point_data = [(p['x'] if 'x' in p else random.uniform(-1, 1), 
                      p['y'] if 'y' in p else random.uniform(-1, 1)) 
                      for p in processed]
        
        # Check if current area is familiar using our area recognition system
        self.current_area_familiar = self.area_recognition.process_lidar_data(point_data)
        
        # Adjust behavior based on area familiarity
        if self.current_area_familiar:
            log_message("FAMILIAR AREA DETECTED - Using efficient navigation mode")
            self.familiar_areas_visited += 1
            # Faster movement, less detailed mapping
            speed_factor = 1.5  # Move faster in familiar areas
        else:
            log_message("NEW AREA DETECTED - Using detailed mapping mode")
            self.new_areas_visited += 1
            # Slower movement with more detailed mapping
            speed_factor = 0.8  # Move slower in new areas
        
        # Plan a random target from current position with area-aware navigation
        current = self.navigation.current_position if self.navigation.current_position else (0, 0)
        target = (current[0] + random.randint(-5, 5), current[1] + random.randint(-5, 5))
        self.navigation.set_target(target)
        path = self.navigation.plan_path(current, target)
        
        # Log different messages based on area familiarity
        if self.current_area_familiar:
            log_message(f"Using efficient path through familiar area: {path}")
        else:
            log_message(f"Planning careful exploration path: {path}")
        
        # Simulate obstacle avoidance and movement with area-aware behavior
        self.navigation.avoid_obstacles()
        self.navigation.move_to_target()
        self.navigation.update_position(target)
        
        # Log exploration statistics
        log_message(f"Position updated to: {target}")
        log_message(f"Familiar areas visited: {self.familiar_areas_visited}, New areas: {self.new_areas_visited}")
        
        # Simulate motor control via GPIO if available with speed adjusted by area familiarity
        if self.gpio_controller is not None:
            # Generate motor speeds adjusted by our familiarity factor
            base_speed = random.uniform(40, 80)
            adjusted_speed = base_speed * speed_factor
            
            # In familiar areas, we might use more symmetric speeds for straighter paths
            if self.current_area_familiar:
                left_speed = adjusted_speed * random.uniform(0.9, 1.0)
                right_speed = adjusted_speed * random.uniform(0.9, 1.0)
            else:
                # More variable speeds in new areas for careful exploration
                left_speed = adjusted_speed * random.uniform(0.7, 1.0)
                right_speed = adjusted_speed * random.uniform(0.7, 1.0)
                
            self.gpio_controller.set_motor_speeds(left_speed, right_speed)
            log_message(f"Set GPIO motor speeds: Left={left_speed:.1f}%, Right={right_speed:.1f}%")

    def stop(self):
        self.running = False
        log_message("Stopping robot")
        
        # Save the final map before shutting down
        self.area_recognition.save_current_map()
        log_message("Final map saved")
        
        if self.gpio_controller is not None:
            self.gpio_controller.cleanup()
        # Additional cleanup if needed

if __name__ == "__main__":
    controller = WaveRoverController()
    controller.start_control_loop()