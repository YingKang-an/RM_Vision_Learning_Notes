# CMake 最低版本要求（需支持 C++20 标准，3.16 及以上即可）
cmake_minimum_required(VERSION 3.16)

# 项目名称与编程语言（明确指定 CXX 即 C++）
project(RoboMaster_Vision_Camera LANGUAGES CXX)

# 强制设置 C++ 标准为 20，不可被后续配置覆盖
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 1. 查找系统中的 OpenCV 库（自动识别系统路径，无需手动配置）
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "错误：未找到 OpenCV 库，请确保 OpenCV 已正确安装")
endif()

# 2. 配置大恒相机 SDK 路径（完全匹配真实路径和库名）
set(GALAXY_SDK_ROOT /opt/Galaxy_camera)
set(GALAXY_INCLUDE_DIR ${GALAXY_SDK_ROOT}/inc)    # 头文件路径（已验证正确）
set(GALAXY_LIB_DIR ${GALAXY_SDK_ROOT}/lib/x86_64) # 修正：真实库路径，包含 x86_64 子目录

# 检查大恒 SDK 关键目录是否存在
if(NOT EXISTS ${GALAXY_INCLUDE_DIR})
    message(FATAL_ERROR "错误：大恒 SDK 头文件路径不存在 -> ${GALAXY_INCLUDE_DIR}")
endif()
if(NOT EXISTS ${GALAXY_LIB_DIR})
    message(FATAL_ERROR "错误：大恒 SDK 库文件路径不存在 -> ${GALAXY_LIB_DIR}")
endif()

# 3. 添加可执行文件（目标名称为 main，源文件为根目录下的 main.cpp）
add_executable(main 
    main.cpp
    # 示例：若有更多源文件，直接追加在此处
    # camera_driver.cpp
    # image_processor.cpp
)

# 4. 配置头文件包含目录（供编译器查找头文件）
target_include_directories(main
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include    # 项目自身的头文件目录（可选，不存在则忽略）
    ${OpenCV_INCLUDE_DIRS}           # OpenCV 的头文件目录（自动获取）
    ${GALAXY_INCLUDE_DIR}            # 大恒 SDK 的头文件目录（inc）
)

# 5. 配置库文件链接目录（供链接器查找库文件）
target_link_directories(main
    PRIVATE
    ${GALAXY_LIB_DIR}                # 大恒 SDK 的真实库路径（lib/x86_64）
)

# 6. 链接所需的库文件（核心修正：库名改为全小写 gxiapi）
target_link_libraries(main
    PRIVATE
    ${OpenCV_LIBS}                   # OpenCV 库（自动获取所有链接库）
    gxiapi                           # 修正：全小写库名，匹配实际库文件 libgxiapi.so
)

# 可选配置：设置运行时库搜索路径，解决程序运行时找不到大恒 SDK 库的问题
set_target_properties(main PROPERTIES
    INSTALL_RPATH ${GALAXY_LIB_DIR}
    BUILD_WITH_INSTALL_RPATH ON
)
