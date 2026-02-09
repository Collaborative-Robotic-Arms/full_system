cmake_minimum_required(VERSION 3.8)
project(abb_endeffector_control)

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
)

# --- 1. DEFINE THE EXECUTABLE ---
# We name the target "abb_endeffector_control_client"
add_executable(abb_endeffector_control_client src/endEffector.cpp)

# --- 2. LINK DEPENDENCIES ---
# UPDATED: Use the SAME name "abb_endeffector_control_client"
ament_target_dependencies(abb_endeffector_control_client 
  rclcpp 
  abb_robot_msgs
) 

# Explicitly link against the generated typesupport library (since srv is in same package)
# UPDATED: Use the SAME name "abb_endeffector_control_client"
target_link_libraries(abb_endeffector_control_client
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# --- 3. INSTALL ---
# UPDATED: Use the SAME name "abb_endeffector_control_client"
install(TARGETS abb_endeffector_control_client
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for the service interface
ament_export_dependencies(rosidl_default_runtime)

ament_package()