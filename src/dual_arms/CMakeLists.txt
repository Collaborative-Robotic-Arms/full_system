cmake_minimum_required(VERSION 3.8)
project(dual_arms)

# =====================================================
# 🔹 Compiler Settings
# =====================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =====================================================
# 🔹 Core Build System
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

# =====================================================
# 🔹 ROS 2 Core Dependencies
# =====================================================
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)

# =====================================================
# 🔹 Gazebo / Ignition
# =====================================================
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# =====================================================
# 🔹 Common Utilities
# =====================================================
find_package(xacro REQUIRED)
find_package(yaml-cpp REQUIRED)

# =====================================================
# 🔹 Message Packages
# =====================================================
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# =====================================================
# 🔹 MoveIt 2 Dependencies
# =====================================================
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# =====================================================
# 🔹 External / Custom Packages
# =====================================================
find_package(ros2srrc_data REQUIRED)
find_package(abb_robot_msgs REQUIRED)
find_package(ros2_robotiqgripper REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2srrc_execution REQUIRED)
find_package(ros2srrc_robots REQUIRED)
find_package(supervisor_package REQUIRED)

# =====================================================
# 🔹 Include Directories (SRRC logic preserved)
# =====================================================
set(SRRC_BASE
  "${CMAKE_CURRENT_SOURCE_DIR}/../abb_package/ros2_SimRealRobotControl/ros2srrc_execution"
)

if(EXISTS "${SRRC_BASE}/include")
  include_directories("${SRRC_BASE}/include")
  message(STATUS "Found SRRC headers at: ${SRRC_BASE}/include")
else()
  include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/../ros2_SimRealRobotControl/ros2srrc_execution/include"
  )
  message(STATUS "Using fallback SRRC header path")
endif()

# =====================================================
# 🔹 Source Directory Resolution (SRRC executables)
# =====================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../abb_package/ros2_SimRealRobotControl/ros2srrc_execution/src")
  set(SRRC_EXEC_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/../abb_package/ros2_SimRealRobotControl/ros2srrc_execution/src"
  )
else()
  set(SRRC_EXEC_SRC
    "${CMAKE_CURRENT_SOURCE_DIR}/../ros2_SimRealRobotControl/ros2srrc_execution/src"
  )
endif()

# =====================================================
# 🔹 Executables
# =====================================================

# --- SRRC-based executables ---
add_executable(robmove ${SRRC_EXEC_SRC}/robmove.cpp)
ament_target_dependencies(
  robmove
  rclcpp
  moveit_ros_planning_interface
  yaml-cpp
  ros2srrc_data
)

add_executable(robpose ${SRRC_EXEC_SRC}/robpose.cpp)
ament_target_dependencies(
  robpose
  rclcpp
  moveit_ros_planning_interface
  yaml-cpp
  ros2srrc_data
)

add_executable(move
  ${SRRC_EXEC_SRC}/moveg.cpp
  ${SRRC_EXEC_SRC}/moverp.cpp
  ${SRRC_EXEC_SRC}/moverot.cpp
  ${SRRC_EXEC_SRC}/mover.cpp
  ${SRRC_EXEC_SRC}/movel.cpp
  ${SRRC_EXEC_SRC}/movej.cpp
  ${SRRC_EXEC_SRC}/move.cpp
)
ament_target_dependencies(
  move
  rclcpp
  moveit_ros_planning_interface
  yaml-cpp
  ros2srrc_data
)
target_link_libraries(move yaml-cpp)

# --- Local project executable ---
add_executable(dual_arms_controller src/dual_arms_controller.cpp)
ament_target_dependencies(
  dual_arms_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

add_executable(ar4_pose_server src/ar4_pose_server.cpp)
ament_target_dependencies(
  ar4_pose_server
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  supervisor_package
)

add_executable(AR4_pose_commander src/AR4_pose_commander.cpp)
ament_target_dependencies(
  AR4_pose_commander
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  supervisor_package
)

# =====================================================
# 🔹 Install Targets
# =====================================================
install(
  TARGETS
  robmove
  move
  robpose
  dual_arms_controller
  AR4_pose_commander
  ar4_pose_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  config
  launch
  srdf
  urdf
  meshes
  worlds
  materials
  models
  DESTINATION share/${PROJECT_NAME}
)

# =====================================================
# 🔹 Export
# =====================================================
ament_export_dependencies(Boost)
ament_package()
