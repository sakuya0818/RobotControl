cmake_minimum_required(VERSION 3.8)
project(demo_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(service_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(turtle_control
  src/turtle_control.cpp
)
add_executable(patrol_client
  src/patrol_client.cpp
)
ament_target_dependencies(turtle_control
  rclcpp
  service_interfaces
  geometry_msgs
  turtlesim
)
ament_target_dependencies(patrol_client
  rclcpp
  service_interfaces
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

install (TARGETS turtle_control patrol_client
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
