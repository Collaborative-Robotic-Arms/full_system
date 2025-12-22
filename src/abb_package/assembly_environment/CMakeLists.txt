cmake_minimum_required(VERSION 3.8) # Updated for ROS 2 compatibility

project(assembly_environment)

# --- ROS 2 Ament Setup ---
find_package(ament_cmake REQUIRED)
# Find essential ROS 2 packages for robot descriptions
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(robot_state_publisher REQUIRED) # Added common dependency

# --- Installation ---
# Install the directories containing configuration and robot description files
install(DIRECTORY
    config
    meshes
    urdf
    DESTINATION share/${PROJECT_NAME}/
)

# Standard ament package marker
ament_package()