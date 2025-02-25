cmake_minimum_required(VERSION 3.0.2)
project(wave_rover_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs nav_msgs geometry_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  src/wave_rover_controller.py
  src/lidar_integration.py
  src/mapping.py
  src/navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/utils/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/utils
)