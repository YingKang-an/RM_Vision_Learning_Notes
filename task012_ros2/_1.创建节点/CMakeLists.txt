# 最低cmake版本要求
cmake_minimum_required(VERSION 3.18)

# 项目名称
project(main)

# 可执行文件生成
add_executable(main ros2_node.cpp)

find_package(rclcpp REQUIRED)
# 打印头文件及头文件依赖的其他头文件
message(STATUS ${rclcpp_INCLUDE_DIRS})
# 打印库文件及库文件依赖的其他库文件
message(STATUS ${rclcpp_LIBRARIES})

# 头文件包含
target_include_directories(${PROJECT_NAME}
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
)
# 库文件链接
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  ${rclcpp_LIBRARIES}
)
