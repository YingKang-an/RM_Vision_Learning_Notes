/**
 * @author      YinKang'an
 * @date        2024-02-01
 * @version     V3.1.0
 * @brief       张正友相机标定程序（优化版本）
 * @description 基于大恒相机的张正友标定方法，修复重投影误差保存错误，优化性能
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <future>
#include <memory>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include "tools/camera.hpp"
#include "tools/calibration_utils.hpp"
#include "tools/image_processor.hpp"

// 全局变量
GX_DEV_HANDLE hDevice = nullptr;
std::vector<std::vector<cv::Point2f>> imagePoints;
std::atomic<bool> isProcessing(false);
std::atomic<int> capturedCount(0);

// 标定参数
struct CalibrationConfig {
  cv::Size boardSize;
  float squareSize;
  int minImages;
};

// 从配置文件读取标定参数
CalibrationConfig loadCalibrationConfig() {
  CalibrationConfig config;
  config.boardSize = cv::Size(9, 6);
  config.squareSize = 25.0f;
  config.minImages = 10;
  
  try {
    YAML::Node yamlConfig = YAML::LoadFile("../config/config.yaml");
    if (yamlConfig["calibration"]) {
      YAML::Node calib = yamlConfig["calibration"];
      if (calib["board_size"]) {
        auto size = calib["board_size"].as<std::vector<int>>();
        if (size.size() == 2) {
          config.boardSize = cv::Size(size[0], size[1]);
        }
      }
      if (calib["square_size"]) {
        config.squareSize = calib["square_size"].as<float>();
      }
      if (calib["min_images"]) {
        config.minImages = calib["min_images"].as<int>();
      }
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading calibration config: " << e.what() << std::endl;
  }
  
  return config;
}

// 读取版本信息
std::string loadVersionInfo() {
  try {
    YAML::Node versionConfig = YAML::LoadFile("../config/version.yaml");
    if (versionConfig["Version"] && versionConfig["Description"]) {
      return "v" + versionConfig["Version"].as<std::string>() + " - " + 
             versionConfig["Description"].as<std::string>();
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading version info: " << e.what() << std::endl;
  }
  return "v3.1.0 - 优化版本";
}

// 执行相机标定
bool performCalibration(const CalibrationConfig& config, 
                       cv::Mat& cameraMatrix, cv::Mat& distCoeffs, 
                       std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                       double& reprojectionError) {
  if (imagePoints.size() < static_cast<size_t>(config.minImages)) {
    std::cout << "标定图像不足，需要至少 " << config.minImages << " 张图像" << std::endl;
    return false;
  }
  
  // 生成世界坐标系角点
  std::vector<std::vector<cv::Point3f>> objectPoints;
  std::vector<cv::Point3f> obj;
  for (int i = 0; i < config.boardSize.height; i++) {
    for (int j = 0; j < config.boardSize.width; j++) {
      obj.push_back(cv::Point3f(j * config.squareSize, i * config.squareSize, 0));
    }
  }
  
  for (size_t i = 0; i < imagePoints.size(); i++) {
    objectPoints.push_back(obj);
  }
  
  // 执行标定
  cv::Size imageSize(640, 480);  // 假设图像尺寸
  reprojectionError = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                  cameraMatrix, distCoeffs, rvecs, tvecs,
                                  cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5);
  
  std::cout << "标定完成，重投影误差: " << reprojectionError << std::endl;
  return true;
}

int main() {
  // 加载标定配置和版本信息
  CalibrationConfig config = loadCalibrationConfig();
  std::string versionInfo = loadVersionInfo();
  
  std::cout << "=== 张正友相机标定程序 " << versionInfo << " ===" << std::endl;
  std::cout << "棋盘格尺寸: " << config.boardSize.width << "x" << config.boardSize.height << std::endl;
  std::cout << "方格大小: " << config.squareSize << "mm" << std::endl;
  std::cout << "最小标定图像: " << config.minImages << " 张" << std::endl;
  
  // 初始化相机
  if (!initializeCamera(hDevice)) {
    return -1;
  }
  
  // 从配置文件加载相机参数
  if (!loadCameraConfig(hDevice, "../config/config.yaml")) {
    std::cerr << "加载相机配置失败，使用默认参数" << std::endl;
  }
  
  // 启动采集
  if (!startCapture(hDevice)) {
    cleanupCamera(hDevice);
    return -1;
  }
  
  // 创建图像处理器（充分利用24核心，使用20个线程）
  ImageProcessor imageProcessor(20);
  
  // 创建窗口
  cv::namedWindow("Camera Calibration", cv::WINDOW_NORMAL);
  cv::namedWindow("Corner Detection", cv::WINDOW_NORMAL);
  
  std::cout << "按键控制:" << std::endl;
  std::cout << "  SPACE - 采集标定图像" << std::endl;
  std::cout << "  C     - 开始标定" << std::endl;
  std::cout << "  S     - 保存标定结果" << std::endl;
  std::cout << "  ESC   - 退出程序" << std::endl;
  
  bool quit = false;
  GX_FRAME_BUFFER* currentFrame = nullptr;
  std::future<ProcessResult> processingFuture;
  ProcessResult currentResult;
  
  auto startTime = std::chrono::high_resolution_clock::now();
  int frameCount = 0;
  double fps = 0.0;
  
  while (!quit) {
    // 获取图像
    uint32_t frameBufferCount;
    GX_FRAME_BUFFER* frames[5];
    GX_STATUS status = GXDQAllBufs(hDevice, frames, 5, &frameBufferCount, 1000);
    
    if (status == GX_STATUS_SUCCESS && frameBufferCount > 0) {
      currentFrame = frames[frameBufferCount - 1];
      if (currentFrame->nStatus == GX_FRAME_STATUS_SUCCESS) {
        cv::Mat img(currentFrame->nHeight, currentFrame->nWidth, CV_8UC1, currentFrame->pImgBuf);
        cv::Mat colorImg;
        cv::cvtColor(img, colorImg, cv::COLOR_BayerBG2BGR);
        
        // 异步处理图像（高性能并行处理）
        if (!isProcessing) {
          isProcessing = true;
          processingFuture = imageProcessor.processImageAsync(colorImg, capturedCount);
        }
        
        // 非阻塞检查处理结果
        if (isProcessing && processingFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
          currentResult = processingFuture.get();
          isProcessing = false;
        }
        
        // 计算FPS
        frameCount++;
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        if (elapsedTime > 500) {  // 每500ms更新一次FPS显示
          fps = frameCount * 1000.0 / elapsedTime;
          frameCount = 0;
          startTime = currentTime;
        }
        
        // 显示主窗口（包含FPS显示）
        cv::Mat displayImage = imageProcessor.drawResult(colorImg, currentResult, capturedCount);
        
        // 在主窗口显示FPS（深色文字，无背景）
        std::string fpsText = "fps: " + std::to_string(static_cast<int>(fps));
        cv::putText(displayImage, fpsText, cv::Point(displayImage.cols - 120, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 30), 2);
        
        cv::imshow("Camera Calibration", displayImage);
        
        // 显示角点检测窗口（使用异步预处理的角点图像）
        cv::Mat cornerDisplayImage;
        if (!currentResult.cornerImage.empty()) {
          cornerDisplayImage = currentResult.cornerImage.clone();
        } else {
          cornerDisplayImage = colorImg.clone();
        }
        
        // 在角点检测窗口也显示FPS（深色文字，无背景）
        cv::putText(cornerDisplayImage, fpsText, cv::Point(cornerDisplayImage.cols - 120, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 30), 2);
        
        cv::imshow("Corner Detection", cornerDisplayImage);
      }
      GXQAllBufs(hDevice);
    }
    
    // 处理键盘输入
    int key = cv::waitKey(1);
    switch (key) {
      case 32:  // SPACE - 采集图像
        if (currentFrame && currentFrame->nStatus == GX_FRAME_STATUS_SUCCESS) {
          cv::Mat temp;
          cv::cvtColor(cv::Mat(currentFrame->nHeight, currentFrame->nWidth, CV_8UC1, currentFrame->pImgBuf), 
                      temp, cv::COLOR_BayerBG2BGR);
          
          std::vector<cv::Point2f> corners;
          if (imageProcessor.detectChessboard(temp, corners)) {
            imagePoints.push_back(corners);
            capturedCount++;
            std::cout << "成功采集图像 " << capturedCount << std::endl;
          } else {
            std::cout << "未检测到棋盘格，请调整位置" << std::endl;
          }
        }
        break;
        
      case 99:  // C - 开始标定
        if (capturedCount >= config.minImages) {
          cv::Mat cameraMatrix, distCoeffs;
          std::vector<cv::Mat> rvecs, tvecs;
          double reprojectionError;
          
          if (performCalibration(config, cameraMatrix, distCoeffs, rvecs, tvecs, reprojectionError)) {
            std::cout << "标定成功!" << std::endl;
            std::cout << "相机内参矩阵:" << std::endl << cameraMatrix << std::endl;
            std::cout << "畸变系数:" << std::endl << distCoeffs << std::endl;
            std::cout << "重投影误差: " << reprojectionError << std::endl;
          }
        } else {
          std::cout << "标定图像不足，需要至少 " << config.minImages << " 张图像" << std::endl;
        }
        break;
        
      case 115: // S - 保存结果
        {
          cv::Mat cameraMatrix, distCoeffs;
          std::vector<cv::Mat> rvecs, tvecs;
          double reprojectionError;
          
          if (performCalibration(config, cameraMatrix, distCoeffs, rvecs, tvecs, reprojectionError)) {
            cv::FileStorage fs("../config/camera_calibration.yaml", cv::FileStorage::WRITE);
            fs << "camera_matrix" << cameraMatrix;
            fs << "distortion_coefficients" << distCoeffs;
            fs << "reprojection_error" << reprojectionError;  // 修复：使用正确的重投影误差
            fs.release();
            std::cout << "标定结果已保存到 ../config/camera_calibration.yaml" << std::endl;
            std::cout << "重投影误差: " << reprojectionError << std::endl;
          }
        }
        break;
        
      case 27:  // ESC - 退出
        quit = true;
        break;
    }
  }
  
  // 清理资源
  cleanupCamera(hDevice);
  cv::destroyAllWindows();
  
  std::cout << "程序退出，共处理 " << capturedCount << " 张图像" << std::endl;
  return 0;
}