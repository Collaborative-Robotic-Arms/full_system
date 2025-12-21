cmake_minimum_required(VERSION 3.10)
project(point_control_pkg)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ==========================================================
# Dependencies
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # <--- Added for Action Server support
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(supervisor_package REQUIRED) # <--- Added to find your custom actions

# ==========================================================
# Include directories
# ==========================================================
include_directories(include)

# Fix for ROS Jazzy control_msgs being nested incorrectly (kept from your original)
if(EXISTS "/opt/ros/jazzy/include/control_msgs/control_msgs/msg/joint_jog.hpp")
  message(WARNING "control_msgs include path appears nested, adding manual include directory.")
  include_directories(/opt/ros/jazzy/include/control_msgs/control_msgs)
endif()

# ==========================================================
# Executables
# ==========================================================
add_executable(pose_commander src/pose_commander.cpp)

# ==========================================================
# Link dependencies
# ==========================================================
# This ensures all headers and library paths are correctly mapped
ament_target_dependencies(pose_commander
  rclcpp
  rclcpp_action                   # <--- Added
  moveit_ros_planning_interface
  tf2_geometry_msgs
  control_msgs
  std_msgs
  geometry_msgs
  supervisor_package              # <--- Added
)

# ==========================================================
# Install targets
# ==========================================================
install(TARGETS
  pose_commander
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/jog_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()