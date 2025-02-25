def log_message(message):
    # Function to log messages to the console
    print(f"[LOG] {message}")

def load_configuration(file_path):
    # Function to load configuration from a file
    import json
    with open(file_path, 'r') as config_file:
        config = json.load(config_file)
    return config

def save_configuration(file_path, config):
    # Function to save configuration to a file
    import json
    with open(file_path, 'w') as config_file:
        json.dump(config, config_file, indent=4)

def process_point_cloud_data(point_cloud):
    # Function to process point cloud data from LiDAR
    processed_data = []
    for point in point_cloud:
        # Example processing: filter points based on distance
        if point['z'] < 1.5:  # Keep points within 1.5 meters
            processed_data.append(point)
    return processed_data

def remember_location(location, memory):
    # Function to remember a previously visited location
    if location not in memory:
        memory.append(location)

def load_map(file_path):
    # Function to load a map from a file
    import pickle
    with open(file_path, 'rb') as map_file:
        map_data = pickle.load(map_file)
    return map_data

def save_map(file_path, map_data):
    # Function to save a map to a file
    import pickle
    with open(file_path, 'wb') as map_file:
        pickle.dump(map_data, map_file)