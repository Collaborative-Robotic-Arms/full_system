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
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# (Optional, keep only if really used)
find_package(supervisor_package REQUIRED)

# ==========================================================
# Generate Action Interfaces
# ==========================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs
)

# This is REQUIRED so the generated interfaces are usable
ament_export_dependencies(rosidl_default_runtime)

# ==========================================================
# Include directories
# ==========================================================
include_directories(include)

# Fix for possible nested control_msgs headers (kept from original)
if(EXISTS "/opt/ros/jazzy/include/control_msgs/control_msgs/msg/joint_jog.hpp")
  message(WARNING "control_msgs include path appears nested, adding manual include directory.")
  include_directories(/opt/ros/jazzy/include/control_msgs/control_msgs)
endif()

# ==========================================================
# Executables
# ==========================================================

# NEW action-based commander
add_executable(pose_commander_action src/pose_commander_action.cpp)

# ==========================================================
# Link dependencies
# ==========================================================

ament_target_dependencies(pose_commander_action
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  tf2_geometry_msgs
  control_msgs
  std_msgs
  geometry_msgs
)

# Link generated action interfaces
rosidl_target_interfaces(pose_commander_action
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ==========================================================
# Install targets
# ==========================================================
install(TARGETS
  pose_commander_action
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/jog_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
