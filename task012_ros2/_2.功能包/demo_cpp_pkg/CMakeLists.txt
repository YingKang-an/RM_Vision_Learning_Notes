cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)  # 项目名称：demo_cpp_pkg

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 可执行文件生成 - 可执行文件名：main
add_executable(main src/demo_ros2_node.cpp)

# 自动添加依赖
ament_target_dependencies(main
  rclcpp
)
# 此时上方命令代替这两句,就不用写了
# 安装头文件（如果有的话）
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

# 安装启动文件（如果有的话）
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )


# 安装可执行文件到lib/demo_cpp_pkg目录
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()