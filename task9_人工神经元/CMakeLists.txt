cmake_minimum_required(VERSION 3.18)

project(perceptron_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -Wextra")
endif()

find_package(OpenCV REQUIRED)

if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "未找到OpenCV库！请配置OpenCV环境变量")
endif()

set(yaml_cpp_INCLUDE_DIRS "/usr/include")
set(yaml_cpp_LIBRARIES "/usr/lib/x86_64-linux-gnu/libyaml-cpp.a")

if(NOT EXISTS ${yaml_cpp_INCLUDE_DIRS}/yaml-cpp/yaml.h)
    message(FATAL_ERROR "yaml-cpp头文件未找到！路径：${yaml_cpp_INCLUDE_DIRS}/yaml-cpp/yaml.h")
endif()
if(NOT EXISTS ${yaml_cpp_LIBRARIES})
    message(FATAL_ERROR "yaml-cpp静态库未找到！路径：${yaml_cpp_LIBRARIES}")
endif()

# 创建两个独立的可执行文件
add_executable(main1 main1_随机梯度下降.cpp)
add_executable(main2 main2_批量梯度下降.cpp)
add_executable(main3 main3_小批量梯度下降.cpp)

# 为每个可执行文件设置包含目录
target_include_directories(main1
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml_cpp_INCLUDE_DIRS}
)

target_include_directories(main2    
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml_cpp_INCLUDE_DIRS}
)

target_include_directories(main3   
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${yaml_cpp_INCLUDE_DIRS}
)
# 为每个可执行文件设置链接库
target_link_libraries(main2
    PRIVATE
    ${OpenCV_LIBS}
    ${yaml_cpp_LIBRARIES}
)

target_link_libraries(main1
    PRIVATE
    ${OpenCV_LIBS}
    ${yaml_cpp_LIBRARIES}
)

target_link_libraries(main3
    PRIVATE
    ${OpenCV_LIBS}
    ${yaml_cpp_LIBRARIES}
)