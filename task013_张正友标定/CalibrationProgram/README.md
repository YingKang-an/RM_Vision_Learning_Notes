# 张正友相机标定程序

## 项目概述
本项目实现基于大恒相机的张正友标定法,用于计算相机内参和畸变系数

## 功能特性
- 实时相机图像采集
- 棋盘格角点自动检测
- 张正友标定算法实现
- 标定结果评估与保存
- 参数可视化调节

## 硬件要求
- 大恒相机 (MER-139-210U3C)
- 棋盘格标定板 (9x6内角点)
- Linux系统

## 软件依赖
- OpenCV 4.6.0
- 大恒相机SDK
- yaml-cpp

## 一键启动
```bash
# 给脚本添加执行权限
chmod +x start.sh
# 正常编译并运行
./start.sh

# 清理后重新编译运行
./start.sh -c
# 仅运行已编译的程序
./start.sh -r
# 显示帮助信息
./start.sh -h
# 显示版本信息
./start.sh -v
```
## 编译运行
```bash
mkdir build
cd build
cmake ..
make
./main
```

## 使用说明
1. 准备9x6的棋盘格标定板
2. 运行程序，调整相机参数使图像清晰
3. 移动标定板到不同位置和角度
4. 按SPACE键采集标定图像
5. 按C键开始标定
6. 按S键保存标定结果

## 文件说明
```bash
CalibrationProgram                        项目根目录
├── CMakeLists.txt                        编译配置文件
├── start.sh                              一键启动脚本
├── config                                配置文件目录
│   ├── config.yaml                       配置文件
│   └── version.yaml                      版本文件
├── main.cpp                              主程序
├── README.md                             项目说明文档
└── tools                                 工具目录
    ├── calibration_utils.cpp             标定算法
    ├── calibration_utils.hpp             标定算法头文件
    ├── camera.cpp                        相机驱动
    ├── camera.hpp                        相机驱动头文件
    ├── image_processor.cpp               图像处理器
    ├── image_processor.hpp               图像处理器头文件
    ├── math_utils.cpp                    数学工具
    ├── math_utils.hpp                    数学工具头文件
    ├── thread_pool.cpp                   线程池
    └── thread_pool.hpp                   线程池头文件
3 directories, 16 files
```