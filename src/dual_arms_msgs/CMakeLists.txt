cmake_minimum_required(VERSION 3.8)
project(dual_arms_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Brick.msg"
  "msg/BricksArray.msg"
  "msg/GraspPoint.msg"
  "msg/SuperBrick.msg"
  "action/MoveToPose.action"
  "action/AlignToTarget.action"
  "action/ExecuteTask.action"
  "srv/GetAssemblyPlan.srv"   
  "srv/DetectBricks.srv"      
  "srv/GetGrasp.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
