cmake_minimum_required(VERSION 3.8)
project(cpp_ros_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 🎯 创建C++节点
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
