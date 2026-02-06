/**
 * @author     YinKang'an
 * @date       2026-02-01
 * @file       camera.hpp
 * @brief      大恒相机驱动头文件
 */

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "GxIAPI.h"
#include <opencv2/opencv.hpp>
#include <string>

// 相机参数结构体
struct CameraParams {
  float exposureVal = 1.0;    // 曝光时间(μs)
  float gainVal = 0.0;         // 增益
  float wbBlue = 0.0;         // 白平衡蓝通道
  float wbGreen = 0.0;        // 白平衡绿通道  
  float wbRed = 0.0;          // 白平衡红通道
};

// 相机控制函数
bool initializeCamera(GX_DEV_HANDLE& device);
bool startCapture(GX_DEV_HANDLE device);
void stopCapture(GX_DEV_HANDLE device);
void cleanupCamera(GX_DEV_HANDLE device);

// 配置读取函数
bool loadCameraConfig(GX_DEV_HANDLE device, const std::string& configPath);
CameraParams getCameraParamsFromConfig(const std::string& configPath);

// 参数设置回调函数
void onExposureChange(int value, void* userdata);
void onGainChange(int value, void* userdata);

#endif // CAMERA_HPP