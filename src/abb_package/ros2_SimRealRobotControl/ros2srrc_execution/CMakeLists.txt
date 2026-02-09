cmake_minimum_required(VERSION 3.8)
project(ros2srrc_execution)

# Enable compiler warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# MoveIt2 dependencies
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Other dependencies
find_package(ros2srrc_data REQUIRED)
find_package(abb_robot_msgs REQUIRED)
find_package(ros2_robotiqgripper REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${YAML_CPP_INCLUDE_DIRS}
  /opt/ros/${ROS_DISTRO}/include
)

# Add executables
add_executable(robmove src/robmove.cpp)
add_executable(move
  src/moveg.cpp
  src/moverp.cpp
  src/moverot.cpp
  src/mover.cpp
  src/movel.cpp
  src/movej.cpp
  src/move.cpp
)
add_executable(robpose src/robpose.cpp)

# Link and depend on libraries
ament_target_dependencies(robmove
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  std_msgs
  yaml-cpp
  ros2srrc_data
)
target_link_libraries(robmove yaml-cpp)

ament_target_dependencies(move
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  std_msgs
  yaml-cpp
  ros2srrc_data
  abb_robot_msgs
  ament_index_cpp
)
target_link_libraries(move yaml-cpp)

ament_target_dependencies(robpose
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  std_msgs
  abb_robot_msgs
  ros2srrc_data
)
target_link_libraries(robpose yaml-cpp)

# C++ executable: move_group_interface_tutorial
add_executable(move_to_pose_node src/move_group_abb.cpp)
ament_target_dependencies(move_to_pose_node
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)

# Install headers, executables, and Python scripts
install(DIRECTORY include/ DESTINATION include)
install(TARGETS move robmove robpose move_to_pose_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY python programs DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  python/RobotState.py
  python/SpawnObject.py
  python/ExecuteProgram.py
  python/testClients/test_Move.py
  python/testClients/test_RobMove.py
  python/testClients/test_PG.py
  python/testClients/test_VG.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
)

ament_package()

