cmake_minimum_required(VERSION 3.8)
project(agv_drive_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agv_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${ignition-common3_INCLUDE_DIRS}
  )

add_library(tricycle_drive_plugin SHARED src/tricycle_drive_plugin.cpp)
ament_target_dependencies(tricycle_drive_plugin
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# Link libraries
target_link_libraries(tricycle_drive_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

add_library(fork_control_plugin SHARED src/fork_control_plugin.cpp)
ament_target_dependencies(fork_control_plugin
  "gazebo_ros"
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "agv_msgs"
)

# Link libraries
target_link_libraries(fork_control_plugin
  ${GAZEBO_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

install(TARGETS
  tricycle_drive_plugin
  fork_control_plugin
  # tricycle_motion_model
  EXPORT export_tricycle_motion_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
