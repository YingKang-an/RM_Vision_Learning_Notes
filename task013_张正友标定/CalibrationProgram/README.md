# 张正友相机标定程序

## 项目概述
本项目实现基于大恒相机的张正友标定法，用于计算相机内参和畸变系数。

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
- OpenCV 4.x
- 大恒相机SDK
- yaml-cpp

## 编译运行
```bash
cd build
cmake ..
make
./calibration_program
```

## 使用说明
1. 准备9x6的棋盘格标定板
2. 运行程序，调整相机参数使图像清晰
3. 移动标定板到不同位置和角度
4. 按SPACE键采集标定图像（至少10张）
5. 按C键开始标定
6. 按S键保存标定结果

## 文件说明
- `main.cpp` - 主程序
- `tools/calibration_utils.cpp/hpp` - 标定算法
- `tools/math_utils.cpp/hpp` - 数学工具
- `tools/camera.cpp/hpp` - 相机驱动
- `config/config.yaml` - 配置文件