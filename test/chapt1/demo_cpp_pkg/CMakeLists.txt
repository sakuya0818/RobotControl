cmake_minimum_required(VERSION 3.8)
project(ros2_test)

# 对 GCC 和 Clang 编译器添加警告选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

# 检查是否找到 ament_cmake
find_package(ament_cmake REQUIRED)
# Find the ROS 2 package
find_package(rclcpp REQUIRED)

set(PROJECT_SOURCES
    src/cpp_node.cpp
)

add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
    rclcpp::rclcpp
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
