cmake_minimum_required(VERSION 3.18)

project(main LANGUAGES CXX)

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

add_executable(${PROJECT_NAME}
    main.cpp
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    # 本地头文件目录（优先查找）
    ${OpenCV_INCLUDE_DIRS}         # OpenCV头文件目录
    ${yaml_cpp_INCLUDE_DIRS}       # yaml-cpp头文件目录
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    # 第三方库
    ${OpenCV_LIBS}                 # OpenCV所有核心库（如core、imgproc、highgui）
    ${yaml_cpp_LIBRARIES}          # yaml-cpp静态库
)


