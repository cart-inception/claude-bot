#!/usr/bin/env python3
"""
Helper functions for the Wave Rover ROS 2 project.
These utilities support various modules with common functionality.
"""
import json
import pickle
import os
from datetime import datetime

def log_message(message):
    """
    Function to log messages to the console with timestamp
    
    Args:
        message (str): The message to log
    """
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] {message}")

def load_configuration(file_path):
    """
    Function to load configuration from a JSON file
    
    Args:
        file_path (str): Path to the configuration file
        
    Returns:
        dict: The loaded configuration
    """
    try:
        with open(file_path, 'r') as config_file:
            config = json.load(config_file)
        return config
    except FileNotFoundError:
        log_message(f"Configuration file not found: {file_path}")
        return {}
    except json.JSONDecodeError:
        log_message(f"Error parsing configuration file: {file_path}")
        return {}

def save_configuration(file_path, config):
    """
    Function to save configuration to a JSON file
    
    Args:
        file_path (str): Path to save the configuration file
        config (dict): Configuration to save
    """
    try:
        with open(file_path, 'w') as config_file:
            json.dump(config, config_file, indent=4)
        log_message(f"Configuration saved to: {file_path}")
    except Exception as e:
        log_message(f"Error saving configuration: {str(e)}")

def process_point_cloud_data(point_cloud):
    """
    Function to process point cloud data from LiDAR
    
    Args:
        point_cloud (list): List of point dictionaries with x, y, z coordinates
        
    Returns:
        list: Processed point cloud data
    """
    processed_data = []
    for point in point_cloud:
        # Example processing: filter points based on distance or any condition
        if 'z' in point and point['z'] < 1.5:  # Keep points within 1.5 meters
            processed_data.append(point)
        elif 'z' not in point:
            # If z not present, include the point anyway
            processed_data.append(point)
    return processed_data

def remember_location(location, memory):
    """
    Function to remember a previously visited location
    
    Args:
        location (tuple): (x, y) coordinates
        memory (list): List of remembered locations
    """
    if location not in memory:
        memory.append(location)
        log_message(f"Remembered location: {location}")

def load_map(file_path):
    """
    Function to load a map from a pickle file
    
    Args:
        file_path (str): Path to the map file
        
    Returns:
        dict: The loaded map data
    """
    try:
        with open(file_path, 'rb') as map_file:
            map_data = pickle.load(map_file)
        return map_data
    except FileNotFoundError:
        log_message(f"Map file not found: {file_path}")
        return None
    except Exception as e:
        log_message(f"Error loading map: {str(e)}")
        return None

def save_map(file_path, map_data):
    """
    Function to save a map to a pickle file
    
    Args:
        file_path (str): Path to save the map file
        map_data (dict): Map data to save
    """
    try:
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
        
        with open(file_path, 'wb') as map_file:
            pickle.dump(map_data, map_file)
        log_message(f"Map saved to: {file_path}")
        
        # Also save metadata in JSON format for easier inspection
        meta_file = os.path.splitext(file_path)[0] + '.json'
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Extract only serializable parts of the map for JSON
            meta_data = {
                'saved_at': timestamp,
                'points_count': len(map_data.get('points', [])),
                'grid_size': map_data.get('grid_size', 0.0),
                'grid_cells_count': len(map_data.get('grid_map', {})) if isinstance(map_data.get('grid_map'), dict) else 0
            }
            
            with open(meta_file, 'w') as f:
                json.dump(meta_data, f, indent=2)
                
        except Exception as e:
            log_message(f"Error saving map metadata: {str(e)}")
            
    except Exception as e:
        log_message(f"Error saving map: {str(e)}")