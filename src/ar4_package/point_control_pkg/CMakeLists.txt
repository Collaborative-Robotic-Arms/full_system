cmake_minimum_required(VERSION 3.10)
project(point_control_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ==========================================================
# Dependencies
# ==========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)              # <--- ADDED
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(supervisor_package REQUIRED)
# 1. Add these to your find_package list
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_warehouse REQUIRED)
find_package(GLUT REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)

# ==========================================================
# Executable
# ==========================================================
add_executable(pose_commander_action
  src/pose_commander.cpp
)

# ==========================================================
# Link ROS dependencies
# ==========================================================
ament_target_dependencies(pose_commander_action
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_perception      # Fixes OccupancyMapMonitor errors
  moveit_ros_warehouse  
  moveit_core
  tf2_ros                                   # <--- ADDED
  tf2_geometry_msgs
  control_msgs
  std_msgs
  geometry_msgs
  supervisor_package
  moveit_ros_occupancy_map_monitor
)

# ==========================================================
# Install
# ==========================================================
install(TARGETS
  pose_commander_action
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jog_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()