cmake_minimum_required(VERSION 3.10)
project(ur3e_tutorials)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf_transformations REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/wiggle.py
  scripts/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()