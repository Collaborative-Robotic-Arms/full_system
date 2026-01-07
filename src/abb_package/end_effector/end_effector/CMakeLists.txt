cmake_minimum_required(VERSION 3.8)
project(end_effector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(abb_robot_msgs REQUIRED) 

# Define the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGripperState.srv"
  # Add this if your .srv uses types from abb_robot_msgs directly
)

# Add the executable for the client node
add_executable(end_effector_client src/endEffector.cpp)

# add_executable(end_effector_clientpy src/endEffectorpy.py)

# Link dependencies for the client node
# The package itself provides the typesupport, so we link to it directly.
ament_target_dependencies(end_effector_client rclcpp abb_robot_msgs) # Removed ${PROJECT_NAME}__rosidl_typesupport_cpp from here

# ament_target_dependencies(end_effector_clientpy rclpy abb_robot_msgs) # Removed ${PROJECT_NAME}__rosidl_typesupport_cpp from here

# Explicitly link against the generated typesupport library
target_link_libraries(end_effector_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Install the executable
install(TARGETS end_effector_client
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for the service interface
ament_export_dependencies(rosidl_default_runtime)

ament_package()