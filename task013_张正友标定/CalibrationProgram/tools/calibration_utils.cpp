/**
 * @file calibration_utils.cpp
 * @brief 相机标定工具函数实现
 */

#include "calibration_utils.hpp"
#include <iostream>
#include <fstream>

namespace calibration {
  
  bool zhangCalibration(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                       const cv::Size& boardSize,
                       float squareSize,
                       const cv::Size& imageSize,
                       CameraParameters& params) {
    if (imagePoints.size() < 10) {
      std::cerr << "标定图像数量不足，至少需要10张图像" << std::endl;
      return false;
    }
    
    // 生成世界坐标系中的角点坐标
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++) {
      for (int j = 0; j < boardSize.width; j++) {
        obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
      }
    }
    
    for (size_t i = 0; i < imagePoints.size(); i++) {
      objectPoints.push_back(obj);
    }
    
    // 执行标定
    std::vector<cv::Mat> rvecs, tvecs;
    params.reprojectionError = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                                  params.cameraMatrix, params.distCoeffs, 
                                                  rvecs, tvecs,
                                                  cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5);
    params.imageSize = imageSize;
    
    std::cout << "标定完成，重投影误差: " << params.reprojectionError << std::endl;
    return true;
  }
  
  void evaluateCalibration(const CameraParameters& params,
                          const std::vector<std::vector<cv::Point2f>>& imagePoints,
                          const cv::Size& boardSize,
                          float squareSize) {
    // 这里可以添加标定质量评估逻辑
    std::cout << "标定质量评估:" << std::endl;
    std::cout << "重投影误差: " << params.reprojectionError << std::endl;
    std::cout << "图像尺寸: " << params.imageSize << std::endl;
  }
  
  bool saveCalibrationResult(const std::string& filename,
                            const CameraParameters& params) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
      std::cerr << "无法创建文件: " << filename << std::endl;
      return false;
    }
    
    fs << "camera_matrix" << params.cameraMatrix;
    fs << "distortion_coefficients" << params.distCoeffs;
    fs << "reprojection_error" << params.reprojectionError;
    fs << "image_width" << params.imageSize.width;
    fs << "image_height" << params.imageSize.height;
    
    fs.release();
    std::cout << "标定结果已保存到: " << filename << std::endl;
    return true;
  }
  
  bool loadCalibrationResult(const std::string& filename,
                            CameraParameters& params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      std::cerr << "无法打开文件: " << filename << std::endl;
      return false;
    }
    
    fs["camera_matrix"] >> params.cameraMatrix;
    fs["distortion_coefficients"] >> params.distCoeffs;
    fs["reprojection_error"] >> params.reprojectionError;
    
    int width, height;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    params.imageSize = cv::Size(width, height);
    
    fs.release();
    std::cout << "标定结果已从 " << filename << " 加载" << std::endl;
    return true;
  }
  
} // namespace calibration