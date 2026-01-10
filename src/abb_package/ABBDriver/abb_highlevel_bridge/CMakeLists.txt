cmake_minimum_required(VERSION 3.8)
project(abb_highlevel_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- FIND DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(abb_robot_msgs REQUIRED)

# MoveIt
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_core REQUIRED)

# CUSTOM ACTION PACKAGE
find_package(supervisor_package REQUIRED)

# --- EXECUTABLE: ABB TASK SERVER ---
add_executable(abb_task_server src/abb_task_server.cpp)
ament_target_dependencies(abb_task_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_ros_move_group
  abb_robot_msgs
  supervisor_package 
)
install(TARGETS abb_task_server DESTINATION lib/${PROJECT_NAME})

# --- (Keep your other executables if needed) ---
add_executable(inverse_kinematics src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics rclcpp moveit_ros_planning_interface moveit_core geometry_msgs tf2_geometry_msgs)
install(TARGETS inverse_kinematics DESTINATION lib/${PROJECT_NAME})

add_executable(abb_endeffector_control_client src/endEffector.cpp)
ament_target_dependencies(abb_endeffector_control_client rclcpp abb_robot_msgs)
install(TARGETS abb_endeffector_control_client DESTINATION lib/${PROJECT_NAME})

add_executable(semantic_publisher src/semantic_publisher.cpp)
ament_target_dependencies(semantic_publisher rclcpp std_msgs)
install(TARGETS semantic_publisher DESTINATION lib/${PROJECT_NAME})

# --- INSTALL LAUNCH FILES ---
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()