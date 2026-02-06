/**
 * @file          camera.cpp
 * @author \b     YinKang'an
 * @date          2026-02-01
 * @version       V2-0-1
 * @brief         大恒相机驱动（优化版本）
 * 
 * @attention \b  [注意]
 *                相机版本: MER-139-210U3C
 */

#include "camera.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>

// 相机初始化
bool initializeCamera(GX_DEV_HANDLE& device) {
  GX_STATUS status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "SDK初始化失败，错误码：" << status << std::endl;
    return false;
  }

  // 枚举设备
  uint32_t deviceNum = 0;
  status = GXUpdateAllDeviceList(&deviceNum, 1000);
  if (deviceNum <= 0) {
    std::cerr << "未检测到相机设备" << std::endl;
    GXCloseLib();
    return false;
  }

  // 打开设备
  GX_OPEN_PARAM openParam;
  openParam.accessMode = GX_ACCESS_EXCLUSIVE;
  openParam.openMode = GX_OPEN_INDEX;
  char content[] = "1";  // 打开第1台相机
  openParam.pszContent = content;
  
  status = GXOpenDevice(&openParam, &device);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "打开相机失败，错误码：" << status << std::endl;
    GXCloseLib();
    return false;
  }

  // 配置相机参数 - 关闭自动模式
  GXSetEnumValueByString(device, "ExposureAuto", "Off");
  GXSetEnumValueByString(device, "GainAuto", "Off");
  GXSetEnumValueByString(device, "BalanceWhiteAuto", "Off");

  return true;
}

// 从配置文件读取相机参数
CameraParams getCameraParamsFromConfig(const std::string& configPath) {
  CameraParams params;
  try {
    YAML::Node config = YAML::LoadFile(configPath);
    if (config["camera"]) {
      YAML::Node camera = config["camera"];
      if (camera["Exposure"]) {
        params.exposureVal = camera["Exposure"].as<float>();
      }
      if (camera["Gain"]) {
        params.gainVal = camera["Gain"].as<float>();
      }
      if (camera["whiteBalance"]) {
        YAML::Node wb = camera["whiteBalance"];
        if (wb["Blue"]) params.wbBlue = wb["Blue"].as<float>();
        if (wb["Green"]) params.wbGreen = wb["Green"].as<float>();
        if (wb["Red"]) params.wbRed = wb["Red"].as<float>();
      }
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading config file: " << e.what() << std::endl;
  }
  return params;
}

// 加载相机配置（优化错误处理）
bool loadCameraConfig(GX_DEV_HANDLE device, const std::string& configPath) {
  try {
    CameraParams params = getCameraParamsFromConfig(configPath);
    
    // 设置曝光
    GX_STATUS status = GXSetFloatValue(device, "ExposureTime", params.exposureVal);
    if (status != GX_STATUS_SUCCESS) {
      std::cerr << "设置曝光失败，错误码：" << status << std::endl;
      return false;
    }
    
    // 设置增益
    status = GXSetFloatValue(device, "Gain", params.gainVal);
    if (status != GX_STATUS_SUCCESS) {
      std::cerr << "设置增益失败，错误码：" << status << std::endl;
      return false;
    }
    
    // 设置白平衡（优化：统一使用GXSetFloatValue）
    status = GXSetEnumValueByString(device, "BalanceRatioSelector", "Blue");
    if (status == GX_STATUS_SUCCESS) {
      GXSetFloatValue(device, "BalanceRatio", params.wbBlue);
    }
    
    status = GXSetEnumValueByString(device, "BalanceRatioSelector", "Green");
    if (status == GX_STATUS_SUCCESS) {
      GXSetFloatValue(device, "BalanceRatio", params.wbGreen);
    }
    
    status = GXSetEnumValueByString(device, "BalanceRatioSelector", "Red");
    if (status == GX_STATUS_SUCCESS) {
      GXSetFloatValue(device, "BalanceRatio", params.wbRed);
    }
    
    std::cout << "Camera parameters loaded from config: Exposure=" << params.exposureVal << "us, Gain=" << params.gainVal << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error loading camera config: " << e.what() << std::endl;
    return false;
  }
}

// 启动采集
bool startCapture(GX_DEV_HANDLE device) {
  GX_STATUS status = GXStreamOn(device);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "启动采集失败，错误码：" << status << std::endl;
    return false;
  }
  return true;
}

// 停止采集
void stopCapture(GX_DEV_HANDLE device) {
  GXStreamOff(device);
}

// 清理相机资源
void cleanupCamera(GX_DEV_HANDLE device) {
  if (device != nullptr) {
    stopCapture(device);
    GXCloseDevice(device);
  }
  GXCloseLib();
}