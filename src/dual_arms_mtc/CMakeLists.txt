cmake_minimum_required(VERSION 3.10)
project(dual_arms_mtc)

# C++17 for modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(supervisor_package REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(dual_arms_msgs REQUIRED)

# ================================================================
# Include directories
# ================================================================
include_directories(include)

# ================================================================
# EXECUTABLE 1: Hybrid MTC Controller Node
# ================================================================
add_executable(hybrid_mtc_controller src/mtc_node.cpp)

target_include_directories(hybrid_mtc_controller PRIVATE include)

ament_target_dependencies(hybrid_mtc_controller
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  supervisor_package
  dual_arms_msgs
)

# ================================================================
# EXECUTABLE 2: Zone Detection Manager
# ================================================================
add_executable(zone_detection_manager src/zone_detection_manager.cpp)

target_include_directories(zone_detection_manager PRIVATE include)

ament_target_dependencies(zone_detection_manager
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  dual_arms_msgs
  moveit_core
  moveit_ros_planning
  moveit_task_constructor_core
)


# ================================================================
# Installation
# ================================================================
install(TARGETS 
  hybrid_mtc_controller
  zone_detection_manager
  DESTINATION lib/${PROJECT_NAME})

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()