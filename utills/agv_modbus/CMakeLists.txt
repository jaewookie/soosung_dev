cmake_minimum_required(VERSION 3.8)
project(agv_modbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

pkg_check_modules(LIBMODBUS REQUIRED IMPORTED_TARGET libmodbus)

include_directories(
  include
  ${LIBMODBUS_INCLUDE_DIRS}
)

add_executable(agv_modbus_test src/agv_com.cpp)
ament_target_dependencies(agv_modbus_test
"rclcpp")
target_link_libraries(agv_modbus_test
  ${LIBMODBUS_LIBRARIES}
  ${rclcpp_LIBRARIES}
)

install(TARGETS
  agv_modbus_test
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
