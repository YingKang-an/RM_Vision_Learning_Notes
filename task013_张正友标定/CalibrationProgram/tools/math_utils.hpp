/**
 * @file math_utils.hpp
 * @brief 数学工具函数
 */

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace math_utils {
    
    /**
     * @brief 计算点到直线的距离
     */
    double pointToLineDistance(const cv::Point2f& point, 
                              const cv::Point2f& lineStart, 
                              const cv::Point2f& lineEnd);
    
    /**
     * @brief 计算多边形的面积
     */
    double polygonArea(const std::vector<cv::Point2f>& points);
    
    /**
     * @brief 计算向量的夹角（弧度）
     */
    double angleBetweenVectors(const cv::Point2f& v1, const cv::Point2f& v2);
    
    /**
     * @brief 坐标变换：图像坐标到世界坐标
     */
    cv::Point3f imageToWorld(const cv::Point2f& imagePoint,
                            const cv::Mat& cameraMatrix,
                            const cv::Mat& rvec,
                            const cv::Mat& tvec,
                            double Z = 0);
    
} // namespace math_utils

#endif // MATH_UTILS_HPP