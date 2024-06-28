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

include_directories(${GAZEBO_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/agv_drive_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "gazebo"
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)
ament_export_libraries(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${nav_msgs_LIBRARIES} ${geometry_msgs_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_geometry_msgs_LIBRARIES})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_plugins.sh.in")

install(TARGETS ${PROJECT_NAME}
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
