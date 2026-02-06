/**
 * @author     YinKang'an
 * @date       2026-02-01
 * @file       image_processor.cpp
 * @brief      图像处理线程管理实现
 */

#include "image_processor.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <algorithm>

// 构造函数
ImageProcessor::ImageProcessor(size_t numThreads) : threadPool(numThreads), boardSize(9, 6), squareSize(25.0f), processedCount(0) {
  
  // 从配置文件读取标定参数
  try {
    YAML::Node config = YAML::LoadFile("../config/config.yaml");
    if (config["calibration"]) {
      YAML::Node calib = config["calibration"];
      if (calib["board_size"]) {
        auto size = calib["board_size"].as<std::vector<int>>();
        if (size.size() == 2) {
          boardSize = cv::Size(size[0], size[1]);
        }
      }
      if (calib["square_size"]) {
        squareSize = calib["square_size"].as<float>();
      }
    }
  } catch (const YAML::Exception& e) {
    std::cerr << "Error loading calibration config: " << e.what() << std::endl;
  }
  
  std::cout << "ImageProcessor initialized with " << numThreads << " threads" << std::endl;
  std::cout << "Board size: " << boardSize.width << "x" << boardSize.height << std::endl;
  std::cout << "Square size: " << squareSize << "mm" << std::endl;
}

// 析构函数
ImageProcessor::~ImageProcessor() {
  threadPool.stop();
}

// 异步处理图像,角点绘制
std::future<ProcessResult> ImageProcessor::processImageAsync(const cv::Mat& image, int imageIndex) {
  return threadPool.enqueue([this, image, imageIndex]() -> ProcessResult {
    ProcessResult result;
    result.imageIndex = imageIndex;
    
    //auto startTime = std::chrono::high_resolution_clock::now();
    
    // 棋盘格检测
    result.chessboardFound = detectChessboard(image, result.corners);
    
    // 预处理图像
    result.processedImage = image.clone();
    
    if (result.chessboardFound) {
      result.cornerImage = image.clone();
      
      // 角点颜色
      std::vector<cv::Scalar> colors = {
        cv::Scalar(255, 0, 0),      // 蓝色
        cv::Scalar(0, 255, 0),      // 绿色  
        cv::Scalar(0, 0, 255),      // 红色
        cv::Scalar(255, 255, 0),    // 青色
        cv::Scalar(255, 0, 255),    // 洋红
        cv::Scalar(0, 255, 255),    // 黄色
        cv::Scalar(128, 0, 128),    // 紫色
        cv::Scalar(255, 165, 0)     // 橙色
      };
      
      // 角点绘制
      for (size_t i = 0; i < result.corners.size(); i++) {
        cv::Point2f corner = result.corners[i];
        cv::Scalar color = colors[i % colors.size()];  /**< 循环使用颜色 */
        // 外圈
        cv::circle(result.cornerImage, corner, 8, color, 2);
        // 内圈
        cv::circle(result.cornerImage, corner, 2, color, -1);
        // 序号显示
        cv::putText(result.cornerImage, std::to_string(i + 1), cv::Point(corner.x + 10, corner.y + 6), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 2);
        cv::putText(result.cornerImage, std::to_string(i + 1), cv::Point(corner.x + 10, corner.y + 6), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(50, 50, 50), 1.5);
      }
      
      // 简化状态信息
      std::string cornerText = "Corners: " + std::to_string(result.corners.size());
      std::string boardText = "Board: " + std::to_string(boardSize.width) + "x" + std::to_string(boardSize.height);
      
      // 状态信息文字（深色，无背景）
      cv::putText(result.cornerImage, "Chessboard Detected", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      cv::putText(result.cornerImage, cornerText, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      cv::putText(result.cornerImage, boardText, cv::Point(10, 75), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                 
    } else {
      // 未检测到棋盘格时的简化显示
      result.cornerImage = image.clone();
      
      // 警告信息
      cv::putText(result.cornerImage, "No chessboard detected", cv::Point(10, 25),cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    
    //auto endTime = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    
    processedCount++;
    
    return result;
  });
}

// 棋盘格检测
bool ImageProcessor::detectChessboard(const cv::Mat& image, std::vector<cv::Point2f>& corners) {
  cv::Mat gray;
  cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  
  // 使用更快的检测参数
  bool found = cv::findChessboardCorners(gray, boardSize, corners, cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_ADAPTIVE_THRESH);
  
  if (found) {
    // 亚像素级精度优化(减少迭代次数提高速度)
    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 0.01));
  }
  
  return found;
}

// 绘制处理结果
cv::Mat ImageProcessor::drawResult(const cv::Mat& image, const ProcessResult& result, int capturedCount) {
  cv::Mat displayImage = image.clone();
  
  // 状态显示
  std::string statusText = "Captured: " + std::to_string(capturedCount);
  std::string threadText = "Threads: " + std::to_string(getThreadCount());
  
  // 状态信息文字
  cv::putText(displayImage, statusText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 70, 70), 2);
  
  // 棋盘格检测状态文字
  if (result.chessboardFound) {
    cv::putText(displayImage, "Chessboard OK", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
  } else {
    cv::putText(displayImage, "No chessboard", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }
  
  // 线程信息文字
  cv::putText(displayImage, threadText, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 70, 70), 2);
  
  return displayImage;
}