cmake_minimum_required(VERSION 3.8)
project(ros2_gui_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- GENERATE MESSAGES (Added GridCell.msg) ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GridCell.msg"
  "msg/Brick.msg"
  "msg/BrickArray.msg"
  DEPENDENCIES geometry_msgs
)

install(PROGRAMS
  scripts/firestore_bridge.py
  scripts/brick_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  scripts/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.json"
)

ament_package()
