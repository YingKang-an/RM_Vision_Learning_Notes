# 最小CMake版本要求
cmake_minimum_required(VERSION 3.16)

# 项目名称
project(CalibrationProgram)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2 -pthread")

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 查找yaml-cpp
find_package(yaml-cpp REQUIRED)

# 配置大恒相机 SDK 路径
set(GALAXY_SDK_ROOT "/opt/Galaxy_camera")
set(GALAXY_INCLUDE_DIR ${GALAXY_SDK_ROOT}/inc)    
set(GALAXY_LIB_DIR ${GALAXY_SDK_ROOT}/lib/x86_64) 

# 检查大恒 SDK 关键目录是否存在
if(NOT EXISTS ${GALAXY_INCLUDE_DIR})
  message(FATAL_ERROR "错误：大恒 SDK 头文件路径不存在 -> ${GALAXY_INCLUDE_DIR}")
endif()
if(NOT EXISTS ${GALAXY_LIB_DIR})
  message(FATAL_ERROR "错误：大恒 SDK 库文件路径不存在 -> ${GALAXY_LIB_DIR}")
endif()

# 添加大恒相机SDK头文件路径
include_directories(${GALAXY_INCLUDE_DIR})
link_directories(${GALAXY_LIB_DIR})

# 设置源文件
set(SOURCES
  main.cpp
  tools/camera.cpp
  tools/calibration_utils.cpp
  tools/math_utils.cpp
  tools/thread_pool.cpp
  tools/image_processor.cpp
)

# 创建可执行文件
add_executable(main ${SOURCES})

# 链接库
target_link_libraries(main 
  ${OpenCV_LIBS}
  yaml-cpp
  gxiapi
  pthread
)

# 包含目录
target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/tools
  ${OpenCV_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${GALAXY_INCLUDE_DIR}
)

# 配置信息
message(STATUS "========== [CalibrationProgram] ==========")
message(STATUS "[Info]OpenCV 版本: ${OpenCV_VERSION}")
message(STATUS "[Info]大恒 SDK 路径: ${GALAXY_SDK_ROOT}")
message(STATUS "[Info]大恒头文件路径: ${GALAXY_INCLUDE_DIR}")
message(STATUS "[Info]大恒库文件路径: ${GALAXY_LIB_DIR}")
message(STATUS "[Info]yaml-cpp 库路径: ${YAML_CPP_LIB}")
message(STATUS "[Info]C++ 标准: ${CMAKE_CXX_STANDARD}")
message(STATUS "[Info]Project configured successfully")
message(STATUS "==========================================")