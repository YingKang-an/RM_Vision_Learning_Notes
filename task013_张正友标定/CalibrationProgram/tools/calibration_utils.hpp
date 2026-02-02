/**
 * @file calibration_utils.hpp
 * @brief 相机标定工具函数
 */

#ifndef CALIBRATION_UTILS_HPP
#define CALIBRATION_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace calibration {
  
  /**
   * @brief 相机标定参数结构体
   */
  struct CameraParameters {
    cv::Mat cameraMatrix;      // 相机内参矩阵
    cv::Mat distCoeffs;        // 畸变系数
    double reprojectionError;  // 重投影误差
    cv::Size imageSize;        // 图像尺寸
    
    CameraParameters() : reprojectionError(0.0) {}
  };
  
  /**
   * @brief 执行张正友标定法
   * @param imagePoints 图像角点坐标
   * @param boardSize 棋盘格尺寸
   * @param squareSize 方格大小(mm)
   * @param imageSize 图像尺寸
   * @param params 输出参数
   * @return 标定是否成功
   */
  bool zhangCalibration(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                       const cv::Size& boardSize,
                       float squareSize,
                       const cv::Size& imageSize,
                       CameraParameters& params);
  
  /**
   * @brief 评估标定质量
   * @param params 相机参数
   * @param imagePoints 图像角点
   * @param boardSize 棋盘格尺寸
   * @param squareSize 方格大小
   */
  void evaluateCalibration(const CameraParameters& params,
                          const std::vector<std::vector<cv::Point2f>>& imagePoints,
                          const cv::Size& boardSize,
                          float squareSize);
  
  /**
   * @brief 保存标定结果
   * @param filename 文件名
   * @param params 相机参数
   * @return 保存是否成功
   */
  bool saveCalibrationResult(const std::string& filename,
                            const CameraParameters& params);
  
  /**
   * @brief 加载标定结果
   * @param filename 文件名
   * @param params 相机参数
   * @return 加载是否成功
   */
  bool loadCalibrationResult(const std::string& filename,
                            CameraParameters& params);
  
} // namespace calibration

#endif // CALIBRATION_UTILS_HPP