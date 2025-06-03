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
# 如果需要使用其他 ROS 2 包，可以取消注释以下行
# find_package(std_msgs REQUIRED)         # ROS 标准消息包，包含基本的消息类型定义
# find_package(geometry_msgs REQUIRED)    # ROS 几何消息包，包含基本的几何消息类型定义
# find_package(sensor_msgs REQUIRED)      # ROS 传感器消息包，包含传感器数据类型定义
# find_package(nav_msgs REQUIRED)         # ROS 导航消息包，包含导航相关的消息类型定义

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
    # std_msgs::std_msgs
    # geometry_msgs::geometry_msgs
    # sensor_msgs::sensor_msgs
    # nav_msgs::nav_msgs
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
