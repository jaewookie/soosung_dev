cmake_minimum_required(VERSION 3.8)
project(agv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(
  include
)

add_executable(agv_drive src/main.cpp src/agv_drive.cpp)

ament_target_dependencies(agv_drive rclcpp geometry_msgs std_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs)

add_executable(agv_drive_by_gpt src/agv_drive_by_gpt.cpp)

ament_target_dependencies(agv_drive_by_gpt rclcpp geometry_msgs std_msgs nav_msgs sensor_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs)

add_executable(agv_drive_fake src/agv_drive_fake.cpp)

ament_target_dependencies(agv_drive_fake rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  agv_drive
  agv_drive_by_gpt
  agv_drive_fake
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
