cmake_minimum_required(VERSION 3.8)
project(abb_endeffector_control)

# Explicitly set C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(abb_robot_msgs REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetGripperState.srv"
  DEPENDENCIES abb_robot_msgs
)

add_executable(end_effector_client src/endEffector.cpp)

# This ensures the executable waits for the service headers to be generated
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
add_dependencies(end_effector_client ${cpp_typesupport_target})

ament_target_dependencies(end_effector_client 
  rclcpp 
  abb_robot_msgs
)

target_link_libraries(end_effector_client
  ${cpp_typesupport_target}
)

install(TARGETS end_effector_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
