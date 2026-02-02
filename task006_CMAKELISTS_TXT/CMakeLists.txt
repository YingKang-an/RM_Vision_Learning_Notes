# ===================== 【基础前置配置：必写项】 =====================
# 1. 指定最低CMake版本要求
#    - 3.18版本是兼顾兼容性和新特性的选择，对C++20、target_*系列现代指令支持完善
#    - CMAKE_MINIMUM_REQUIRED是CMake的入口指令，必须放在最开头
#    - VERSION指定最低版本，低于此版本的CMake会直接终止并报错，避免语法兼容问题
cmake_minimum_required(VERSION 3.18)

# 2. 定义项目核心信息
#    - PROJECT指令作用：① 生成${PROJECT_NAME}变量（值为main），后续所有目标都复用该变量
#                      ② LANGUAGES CXX：仅启用C++编译器，禁用C编译器（减少编译冗余）
#                      ③ 隐含设置CMAKE_CXX_COMPILER等编译器变量
#    - 最佳实践：项目名只在这一处定义，后续修改只需改这里
project(main LANGUAGES CXX)

# 3. 配置C++语言标准（RoboMaster项目推荐C++20）
#    - CMAKE_CXX_STANDARD：指定使用C++20标准（支持concept、coroutine等新特性）
#    - CMAKE_CXX_STANDARD_REQUIRED ON：强制要求编译器支持C++20，不支持则报错（避免降级到低版本）
#    - CMAKE_CXX_EXTENSIONS OFF：禁用编译器非标准扩展（如GNU的__int128），保证跨平台兼容性
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 4. 设置默认编译模式（Release/Debug二选一）
#    - CMAKE_BUILD_TYPE：控制编译器优化等级和调试信息生成
#    - 未手动指定时（如cmake .. 未加-DCMAKE_BUILD_TYPE=Debug），默认用Release模式
#    - Release：生产环境，优化优先；Debug：开发环境，调试优先
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 5. 精细化配置编译器编译选项
#    - CMAKE_CXX_FLAGS：全局C++编译选项，不同模式叠加不同参数
#    - Release模式：-O3（最高级代码优化），减小可执行文件体积+提升运行速度
#    - Debug模式：-g（生成调试信息，供GDB/VSCode调试），-O0（无优化，保证调试准确性）
#    - -Wall/-Wextra：开启编译器基础+额外警告（如未初始化变量、类型不匹配），提前发现代码隐患
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -Wextra")
endif()

# ===================== 【依赖库查找：分手动/自动两种方式】 =====================
# -------------------- 方式1：手动查找 大恒SDK（非标准CMake库，无Config.cmake） --------------------
# 6. 定义大恒SDK根目录变量（带持久化和校验属性）
#    - CACHE PATH：将变量存入build/CMakeCache.txt，支持后续通过cmake -DDAHENG_SDK_PATH=xxx修改
#    - "DaHeng SDK installation path"：变量描述，执行cmake -LA时可查看，提升可读性
#    - 默认路径/opt/Galaxy_camera：Linux下大恒SDK的典型安装路径
set(DAHENG_SDK_PATH "/opt/Galaxy_camera" CACHE PATH "DaHeng SDK installation path")

# 7. 校验大恒SDK根目录有效性（防路径写错导致后续编译失败）
#    - EXISTS：CMake的路径判断函数，检查目录是否存在
#    - FATAL_ERROR：终止CMake流程并打印错误信息，强制用户修正路径
if(NOT EXISTS ${DAHENG_SDK_PATH})
    message(FATAL_ERROR "大恒SDK根目录不存在！请检查路径：${DAHENG_SDK_PATH}")
endif()

# 8. 定义大恒SDK头文件目录（编译器查找.h/.hpp的路径）
#    - inc是大恒SDK的标准头文件目录，存放GxIAPI.h等核心头文件
#    - 后续通过target_include_directories将该路径传给编译器
set(DAHENG_INCLUDE_DIRS
    ${DAHENG_SDK_PATH}/inc
)

# 9. 定义大恒SDK库文件目录（链接器查找.so/.a的路径）
#    - lib/x86_64：Linux 64位系统的库文件目录，存放libgxiapi.so动态库
#    - 大恒SDK的库不在系统默认路径，需手动指定链接路径
set(DAHENG_LINK_DIRS
    ${DAHENG_SDK_PATH}/lib/x86_64
)

# 10. 定义大恒SDK核心库名（链接器的简写代号）
#     - Linux库文件命名规则：lib + 库名 + .so/.a → libgxiapi.so
#     - 链接时只需写gxiapi，链接器会自动补全lib前缀和.so后缀
#     - 若链接静态库，需写完整路径（如${DAHENG_LINK_DIRS}/libgxiapi.a）
set(DAHENG_LIBS gxiapi)

# -------------------- 方式2：自动查找 OpenCV（符合CMake标准，有Config.cmake） --------------------
# 11. 自动查找OpenCV库（现代CMake推荐方式）
#     - find_package：CMake内置指令，自动查找符合标准的第三方库
#     - REQUIRED：找不到OpenCV则终止CMake，避免后续编译报错
#     - 找到后自动生成：OpenCV_INCLUDE_DIRS（头文件路径）、OpenCV_LIBS（所有库名）
#     - 前提：OpenCV已安装且环境变量OpenCV_DIR指向其Config.cmake所在目录
find_package(OpenCV REQUIRED)
# 二次校验（增强鲁棒性，部分环境find_package返回true但实际未找到）
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "未找到OpenCV库！请配置OpenCV环境变量")
endif()

# -------------------- 方式3：手动查找 yaml-cpp（指定静态库完整路径） --------------------
# 12. 定义yaml-cpp头文件目录（系统默认安装路径）
#     - /usr/include：Linux系统默认头文件目录，yaml-cpp的头文件存放在此
#     - 若手动安装yaml-cpp到/opt/yaml，需改为/opt/yaml/include
set(yaml_cpp_INCLUDE_DIRS "/usr/include")

# 13. 定义yaml-cpp静态库完整路径（编译时打包进可执行文件）
#     - .a是静态库，区别于.so动态库：静态库无运行时依赖，适合比赛/部署场景
#     - /usr/lib/x86_64-linux-gnu：Linux 64位系统默认库目录
#     - 若用动态库，可改为/usr/lib/x86_64-linux-gnu/libyaml-cpp.so
set(yaml_cpp_LIBRARIES "/usr/lib/x86_64-linux-gnu/libyaml-cpp.a")

# 14. 校验yaml-cpp头文件和库文件有效性（防安装不完整）
#     - 检查头文件：确保yaml-cpp的核心头文件存在
#     - 检查库文件：确保静态库文件存在，避免链接阶段报错
if(NOT EXISTS ${yaml_cpp_INCLUDE_DIRS}/yaml-cpp/yaml.h)
    message(FATAL_ERROR "yaml-cpp头文件未找到！路径：${yaml_cpp_INCLUDE_DIRS}/yaml-cpp/yaml.h")
endif()
if(NOT EXISTS ${yaml_cpp_LIBRARIES})
    message(FATAL_ERROR "yaml-cpp静态库未找到！路径：${yaml_cpp_LIBRARIES}")
endif()

# ===================== 【现代CMake核心流程：先定义目标，再配配置】 =====================
# 15. 生成可执行文件目标（编译阶段核心）
#     - add_executable：将指定的.cpp源文件编译为可执行文件
#     - ${PROJECT_NAME}：复用项目名作为可执行文件名，统一管理
#     - 源文件列表：1_main.cpp（主函数）、calculate.cpp（计算逻辑），可继续添加detect.cpp等
#     - 注意：头文件无需添加，编译器通过#include自动查找
add_executable(${PROJECT_NAME}
    1_main.cpp
    calculate.cpp
    # 扩展示例：detect.cpp track.cpp camera.cpp
)

# 16. 配置目标的头文件搜索路径（编译器阶段）
#     - target_include_directories：现代CMake指令，仅作用于当前目标（避免全局污染）
#     - PRIVATE：这些头文件仅当前目标使用，不传递给依赖该目标的其他目标
#     - 路径优先级：CMAKE_CURRENT_SOURCE_DIR（本地头文件）> 第三方库头文件
#     - CMAKE_CURRENT_SOURCE_DIR：CMakeLists.txt所在目录，用于查找本地自定义头文件（如calculate.hpp）
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}    # 本地头文件目录（优先查找）
    ${DAHENG_INCLUDE_DIRS}         # 大恒SDK头文件目录
    ${OpenCV_INCLUDE_DIRS}         # OpenCV头文件目录
    ${yaml_cpp_INCLUDE_DIRS}       # yaml-cpp头文件目录
)

# 17. 配置目标的库文件搜索路径（链接器阶段前置）
#     - target_link_directories：现代CMake指令，替代老旧的link_directories（全局污染）
#     - PRIVATE：仅当前目标使用，避免影响其他目标
#     - 仅需配置非系统默认路径的库：大恒SDK在/opt下，需手动指定；OpenCV/yaml-cpp在系统默认路径，无需配置
#     - 链接器会在此路径下查找DAHENG_LIBS对应的libgxiapi.so
target_link_directories(${PROJECT_NAME}
    PRIVATE
    ${DAHENG_LINK_DIRS}
)

# 18. 链接所有依赖库（链接阶段核心）
#     - target_link_libraries：现代CMake核心指令，必须写在add_executable之后
#     - PRIVATE：库仅当前目标使用，不传递给依赖目标
#     - 链接顺序：第三方库（大恒/OpenCV/yaml-cpp）→ 系统库（pthread/dl）
#     - pthread：线程库（视觉项目多线程必须加，如相机回调、图像处理线程）
#     - dl：动态库加载库（大恒SDK内部依赖，解决动态库符号解析问题）
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    # 第三方库
    ${DAHENG_LIBS}                 # 大恒SDK动态库
    ${OpenCV_LIBS}                 # OpenCV所有核心库（如core、imgproc、highgui）
    ${yaml_cpp_LIBRARIES}          # yaml-cpp静态库
    # 系统依赖库（Linux必备）
    pthread                        # 线程库
    dl                             # 动态库加载库
)

# ===================== 【运行时依赖修复：解决动态库找不到的问题】 =====================
# 19. 配置运行时库搜索路径（RPATH）
#     - 问题：编译通过但运行时提示"找不到libgxiapi.so"，因为/opt不在系统默认库路径
#     - 解决方案：给可执行文件嵌入RPATH，运行时自动去指定目录找动态库
#     - INSTALL_RPATH：指定运行时库搜索路径（大恒SDK库目录）
#     - BUILD_WITH_INSTALL_RPATH ON：编译时就将RPATH嵌入可执行文件，无需后续安装
set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH ${DAHENG_LINK_DIRS}
    BUILD_WITH_INSTALL_RPATH ON
)

# 20. 可选配置：指定可执行文件输出目录（提升工程整洁度）
#     - set_target_properties：针对单个目标配置属性，优先级高于全局变量
#     - RUNTIME_OUTPUT_DIRECTORY：指定可执行文件的输出目录（仅针对当前目标）
#     - ${CMAKE_BINARY_DIR}/bin：编译产物输出到build/bin目录
#     - 补充：若需区分Release/Debug，可加 RUNTIME_OUTPUT_DIRECTORY_RELEASE/DEBUG
set_target_properties(${PROJECT_NAME} PROPERTIES
    # 基础配置：所有模式都输出到build/bin
   RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    # 可选：区分Release/Debug目录（比如build/bin/Release、build/bin/Debug）
    # RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/Release
    # RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin/Debug
)
