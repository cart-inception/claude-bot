cmake_minimum_required(VERSION 3.5)
project(wave_rover_ros)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/wave_rover_controller.py
  src/lidar_integration.py
  src/mapping.py
  src/navigation.py
  src/area_recognition.py
  src/gpio_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install utility files
install(DIRECTORY
  src/utils/
  DESTINATION lib/${PROJECT_NAME}/utils
)

# Install config files
install(FILES
  config.json
  DESTINATION share/${PROJECT_NAME}
)

ament_package()