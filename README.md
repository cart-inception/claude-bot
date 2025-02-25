# Claude Bot ROS

## Overview

Claude Bot ROS is a robotics software suite designed for autonomous navigation and control of a rover. This project integrates various modules that offer functionalities ranging from sensor data processing to real-time decision making.

## Key Features and Implementation Details

### 1. Area Recognition
- **Feature:** Identifies and classifies geographical or operational areas using sensor data.
- **Implementation:**
  - Utilizes the logic in `src/area_recognition.py` to process visual and environmental data.
  - Implements algorithms that analyze sensor patterns to determine different terrain types or obstacles.
  - Provides real-time feedback for navigation adjustments.

### 2. GPIO Controller
- **Feature:** Handles communication with hardware components via General Purpose Input/Output pins.
- **Implementation:**
  - Found in `src/gpio_controller.py`, this module initializes and controls GPIO pins for sensor input and actuator output.
  - Interfaces directly with motor drivers and other hardware to manage rover movement and status signals.

### 3. Lidar Integration
- **Feature:** Aggregates and processes Lidar sensor data to facilitate obstacle detection and distance measurement.
- **Implementation:**
  - Implemented in `src/lidar_integration.py`, it captures Lidar data and transforms it into actionable spatial information.
  - Works in tandem with mapping and navigation modules for precise obstacle avoidance.

### 4. Mapping
- **Feature:** Constructs and updates a digital map of the environment using collected sensor data.
- **Implementation:**
  - Resides in `src/mapping.py`, using real-time sensor inputs to generate a live map of the rover's surroundings.
  - Supports route planning and dynamic re-routing by maintaining up-to-date environmental layouts.

### 5. Navigation
- **Feature:** Plans and adjusts routes to navigate the rover towards set targets while avoiding obstacles.
- **Implementation:**
  - Detailed in `src/navigation.py`, this module calculates optimal paths based on mapping data and real-time sensor feedback.
  - Incorporates algorithms for dynamic path reconfiguration when encountering unexpected obstacles.

### 6. Claude Bot Controller
- **Feature:** Serves as the central orchestrator, integrating all subsystems into a cohesive, autonomous unit.
- **Implementation:**
  - Located in `src/claude_bot_controller.py`, it manages the coordination between sensor input, data processing, and control outputs.
  - Oversees system-level control loops and decision-making processes to ensure unified operation.

### 7. Utility Functions
- **Feature:** Provides auxiliary functions used across the system for tasks such as data formatting, logging, and routine computations.
- **Implementation:**
  - Found in `src/utils/helper.py`, these utilities are designed to simplify common tasks and promote code reuse.

## Configuration and Launch

- **Configuration:** System parameters (e.g., sensor thresholds, control strategies) are defined in `config.json`.
- **Build and Dependency Management:**
  - Managed with CMake as described in `CMakeLists.txt` and the ROS package manifest in `package.xml`.
- **Launch:** The `launch/claude_bot.launch` file orchestrates the initialization of all components, ensuring proper startup sequence.

## Conclusion

Claude Bot ROS is built around modular design principles to ensure high maintainability and scalability, making it suitable for advanced autonomous rover applications.

## Project Structure

```
claude_bot_ros
├── launch
│   └── claude_bot.launch        # Launch configuration for ROS nodes
├── src
│   ├── claude_bot_controller.py  # Main controller for the CLAUDE BOT
│   ├── lidar_integration.py      # LiDAR integration for point cloud processing
│   ├── mapping.py                # Mapping functionality for creating and updating maps
│   ├── navigation.py             # Navigation and path planning for the robot
│   ├── area_recognition.py       # Area recognition for identifying and classifying areas
│   ├── gpio_controller.py        # GPIO controller for hardware communication
│   └── utils
│       └── helper.py            # Utility functions for data processing and logging
├── package.xml                   # Package configuration for ROS
├── CMakeLists.txt                # Build configuration for ROS
└── README.md                     # Project documentation
```

## Detailed Setup Instructions for Raspberry Pi 5

### 1. Hardware Setup
1. **Prepare the Raspberry Pi 5**:
   - Install Ubuntu Server 20.04 (64-bit) on a microSD card (32GB or larger recommended)
   - Connect the Raspberry Pi to a display, keyboard and mouse for initial setup
   - Ensure the Pi has internet access for package downloads

2. **Connect the Claude Bot Hardware**:
   - Mount the Raspberry Pi 5 on the Claude Bot chassis using the designated mounting plate
   - Connect the LiDAR scanner to `/dev/ttyUSB0` (typically)
   - Connect the Claude Bot controller board to `/dev/ttyUSB1` (typically)
   - Ensure the GPIO pins are properly connected for motor control if using direct GPIO control:
     - Connect left motor control to designated GPIO pins
     - Connect right motor control to designated GPIO pins

3. **Power Setup**:
   - Ensure the 3x 18650 batteries are properly installed in the Claude Bot
   - Connect the Raspberry Pi to a stable 5V power supply (using the Claude Bot's onboard power)
   - Verify that the robot's battery voltage is displaying correctly on the OLED screen

### 2. Software Installation

1. **Install ROS on Raspberry Pi 5**:
   ```bash
   # Add ROS repository
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   
   # Add ROS keys
   sudo apt install curl
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   
   # Update and install ROS Noetic (compatible with Pi 5)
   sudo apt update
   sudo apt install -y ros-noetic-desktop
   
   # Initialize rosdep
   sudo apt install -y python3-rosdep
   sudo rosdep init
   rosdep update
   
   # Setup environment
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   
   # Install additional ROS packages
   sudo apt install -y ros-noetic-rosserial-python ros-noetic-pcl-ros ros-noetic-navigation ros-noetic-tf
   ```

2. **Install Required System Dependencies**:
   ```bash
   sudo apt install -y python3-pip git cmake build-essential
   sudo pip3 install numpy pyserial
   ```

3. **Install Python Dependencies**:
   ```bash
   pip3 install numpy matplotlib pyserial
   sudo apt install -y python3-rpi.gpio  # GPIO library for Raspberry Pi
   ```

### 3. Claude Bot ROS Package Setup

1. **Create ROS Workspace**:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```

2. **Clone the Claude Bot Repository**:
   ```bash
   git clone https://github.com/your-username/claude_bot_ros.git
   ```
   
3. **Build the Package**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Configure Serial Ports**:
   - Verify your USB device paths:
   ```bash
   ls -l /dev/ttyUSB*
   ```
   - If necessary, update the `config.json` file with the correct port paths:
   ```bash
   cd ~/catkin_ws/src/claude_bot_ros
   nano config.json
   ```
   - Default configuration:
   ```json
   {
       "lidar_port": "/dev/ttyUSB0",  
       "rover_control_port": "/dev/ttyUSB1"
   }
   ```
   
5. **Add GPIO Configuration** (if needed):
   - Edit `config.json` to include GPIO pin settings:
   ```json
   {
       "lidar_port": "/dev/ttyUSB0",
       "rover_control_port": "/dev/ttyUSB1",
       "gpio_left_motor_pin": 17,
       "gpio_right_motor_pin": 18
   }
   ```

6. **Set Serial Port Permissions**:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod a+rw /dev/ttyUSB0
   sudo chmod a+rw /dev/ttyUSB1
   ```
   Note: You'll need to log out and back in for group changes to take effect.

### 4. Testing the Installation

1. **Verify ROS Installation**:
   ```bash
   roscore
   ```
   This should start the ROS master without errors.
   
2. **Test Individual Nodes**:
   ```bash
   # In separate terminals:
   rosrun claude_bot_ros lidar_integration.py
   rosrun claude_bot_ros mapping.py
   rosrun claude_bot_ros navigation.py
   ```

3. **Test GPIO Control** (if using GPIO):
   ```bash
   # Minimal test script
   python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(17, GPIO.OUT); GPIO.output(17, GPIO.HIGH); import time; time.sleep(1); GPIO.output(17, GPIO.LOW); GPIO.cleanup()"
   ```

### 5. Running the System

1. **Launch the Complete System**:
   ```bash
   roslaunch claude_bot_ros claude_bot.launch
   ```

2. **Auto-start on Boot** (Optional):
   - Create a startup service:
   ```bash
   sudo nano /etc/systemd/system/claude-bot.service
   ```
   - Add the following content:
   ```
   [Unit]
   Description=Claude Bot ROS
   After=network.target

   [Service]
   User=pi
   WorkingDirectory=/home/pi/catkin_ws
   ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash && source /home/pi/catkin_ws/devel/setup.bash && roslaunch claude_bot_ros claude_bot.launch"
   Restart=on-failure
   RestartSec=5

   [Install]
   WantedBy=multi-user.target
   ```
   - Enable the service:
   ```bash
   sudo systemctl enable claude-bot.service
   ```

### 6. Troubleshooting

1. **Serial Port Issues**:
   - If the serial ports are not recognized correctly:
   ```bash
   ls -l /dev/ttyUSB*  # Check available ports
   sudo dmesg | grep tty  # Check device connections
   ```
   - Update the `config.json` with correct paths

2. **Permission Issues**:
   - If you encounter permission errors:
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB1
   ```

3. **GPIO Issues**:
   - Check if GPIO is functioning:
   ```bash
   gpio -v  # Check if GPIO is available
   sudo apt install -y wiringpi  # Alternative GPIO library
   ```

4. **ROS Connection Issues**:
   - Verify ROS networking:
   ```bash
   echo $ROS_MASTER_URI
   # Should be http://localhost:11311 or your Pi's IP
   ```

5. **Log Files**:
   - Check ROS logs for errors:
   ```bash
   cat ~/.ros/log/latest/rosout.log
   ```

## Usage Guidelines

- **Launching the Nodes**: Use the provided launch file to start all necessary nodes for the CLAUDE BOT.

   ```bash
   roslaunch claude_bot_ros claude_bot.launch
   ```

- **Controlling the Robot**: Interact with the robot through the `claude_bot_controller.py` script, which handles user commands and manages the robot's operations.

- **LiDAR Integration**: The `lidar_integration.py` script processes point cloud data from the LiDAR scanner and publishes it to a ROS topic for further processing.

- **Mapping and Navigation**: Utilize the `mapping.py` and `navigation.py` scripts to create and update maps, as well as to plan paths and navigate the robot to target locations.

## Main Functionalities

- **Claude Bot Controller**: Manages the overall operations of the robot, including initialization and command handling.
- **LiDAR Integration**: Processes and publishes LiDAR data for mapping and navigation.
- **Mapping**: Creates and updates maps based on LiDAR data, with functionality to save and load maps.
- **Navigation**: Implements path planning and obstacle avoidance to navigate the robot effectively.

## Contributing

Contributions to enhance the functionality and performance of the CLAUDE BOT ROS integration are welcome. Please submit a pull request or open an issue for discussion.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.