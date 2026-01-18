cmake_minimum_required(VERSION 3.18)
project(main LANGUAGES CXX)

# C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找yaml-cpp或直接指定路径
set(yaml_cpp_INCLUDE_DIRS "/usr/include")
set(yaml_cpp_LIBRARIES "/usr/lib/x86_64-linux-gnu/libyaml-cpp.a")

# 校验yaml-cpp
if(NOT EXISTS ${yaml_cpp_INCLUDE_DIRS}/yaml-cpp/yaml.h)
    message(FATAL_ERROR "yaml-cpp头文件未找到！")
endif()
if(NOT EXISTS ${yaml_cpp_LIBRARIES})
    message(FATAL_ERROR "yaml-cpp静态库未找到！")
endif()

# 生成可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 配置头文件和链接库
target_include_directories(${PROJECT_NAME} PRIVATE ${yaml_cpp_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${yaml_cpp_LIBRARIES})
