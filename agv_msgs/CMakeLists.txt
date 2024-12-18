################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(agv_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# install(
#   DIRECTORY
#   msg
#   DESTINATION
#     share/${PROJECT_NAME}
# )

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ForkControl.msg"
  "msg/AgvDrive.msg"
  "msg/AgvState.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
