cmake_minimum_required(VERSION 3.8)
project(agv_test_pack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agv_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(agv_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(agv_test src/agv_test.cpp)
ament_target_dependencies(agv_test
"rclcpp"
"geometry_msgs"
"std_msgs"
"action_msgs"
"agv_msgs")
target_link_libraries(agv_test
  ${rclcpp_LIBRARIES}
)

add_executable(agv_pal_test src/agv_pal_test.cpp)
ament_target_dependencies(agv_pal_test
"rclcpp"
"geometry_msgs"
"std_msgs"
"action_msgs"
"gazebo_msgs"
"agv_msgs"
"sensor_msgs")
target_link_libraries(agv_pal_test
  ${rclcpp_LIBRARIES}
)


add_executable(agv_can_bus src/agv_can_bus.cpp)

TARGET_LINK_LIBRARIES(agv_can_bus dl -lboost_system -pthread  -lcrypt -lm -lpthread)

ament_target_dependencies(agv_can_bus rclcpp std_msgs can_msgs geometry_msgs agv_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs)

add_executable(agv_rs_imu src/agv_rs_imu.cpp)

TARGET_LINK_LIBRARIES(agv_rs_imu dl -lboost_system -pthread  -lcrypt -lm -lpthread)

ament_target_dependencies(agv_rs_imu rclcpp std_msgs sensor_msgs)

install(TARGETS
  agv_test
  agv_pal_test
  agv_can_bus
  agv_rs_imu
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

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
