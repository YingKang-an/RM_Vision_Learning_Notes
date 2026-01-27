/** 
 * @file   _main.cpp
 * @brief  相机控制主程序
 * 
 * 演示如何使用GxCamera类控制大恒相机，通过OpenCV界面实时调节参数。
 * 
 * @mainpage 大恒相机控制程序
 * 
 * @section intro 介绍
 * 本程序展示如何使用大恒GxIAPI SDK控制工业相机，通过OpenCV界面
 * 实时显示图像并调节相机参数。
 * 
 * @section usage 使用方法
 * @code
 * ./camera_control [device_index]
 * 
 * 参数：
 *   device_index - 相机设备索引，默认1
 * 
 * 操作：
 *   ESC键 - 退出程序
 *   滑动条 - 调节相机参数
 * @endcode
 * 
 * @section modules 模块说明
 * - GxCamera: 相机控制核心类
 * - ImageViewer: OpenCV显示界面
 * - 回调函数: 参数调节响应
 */

#include "GxIAPI.hpp"
#include "opencv.hpp"
#include <iostream>

/**
 * @brief 主函数入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码：0-成功，其他-失败
 * 
 * 程序流程：
 * 1. 解析命令行参数
 * 2. 创建相机对象
 * 3. 打开相机设备
 * 4. 创建显示界面
 * 5. 设置参数调节滑动条
 * 6. 主循环采集和显示图像
 * 7. 清理资源退出
 */
int main(int argc, char* argv[]) {
    std::cout << "=== 大恒相机控制程序 ===" << std::endl;
    std::cout << "按ESC键退出程序" << std::endl;
    
    // 解析设备索引
    int deviceIndex = 1;
    if (argc > 1) {
        deviceIndex = std::atoi(argv[1]);
    }
    
    try {
        // 创建相机对象
        GxCamera camera;
        
        // 打开相机
        if (!camera.open(deviceIndex)) {
            std::cerr << "无法打开相机，设备索引: " << deviceIndex << std::endl;
            return -1;
        }
        
        // 创建显示窗口
        ImageViewer viewer("大恒相机");
        
        // 设置滑动条回调
        viewer.addExposureTrackbar(camera.getExposure(), 100000,
            [&camera](int val) { camera.setExposure(val); });
        
        viewer.addGainTrackbar(camera.getGain() * 10, 1000,
            [&camera](int val) { camera.setGain(val / 10.0); });
        
        // 主循环
        while (!viewer.shouldQuit()) {
            // 采集图像
            cv::Mat frame = camera.capture(1000);
            if (frame.empty()) {
                std::cerr << "采集失败" << std::endl;
                break;
            }
            
            // 显示图像
            viewer.show(frame);
        }
        
        // 关闭相机
        camera.close();
        
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "程序正常退出" << std::endl;
    return 0;
}
