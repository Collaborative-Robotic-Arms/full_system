cmake_minimum_required(VERSION 3.10)
project(supervisor_package)

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# 2. Generate the Actions (.action files)
# This automatically handles the Python and C++ bindings

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/SuperBrick.msg"  
"action/MoveToPose.action"
  "action/AlignToTarget.action"
  "action/ExecuteTask.action"
  "srv/GetAssemblyPlan.srv"   
  "srv/DetectBricks.srv"      
  DEPENDENCIES geometry_msgs
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


# Install the main supervisor
install(PROGRAMS
  supervisor_package/supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME supervisor
)

# Install the mock system
install(PROGRAMS
  supervisor_package/mock_system.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_system
)

# 4. Export and Package
ament_export_dependencies(rosidl_default_runtime)
ament_package()