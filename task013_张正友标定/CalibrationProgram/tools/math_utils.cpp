/**
 * @file math_utils.cpp
 * @brief 数学工具函数实现
 */

#include "math_utils.hpp"
#include <cmath>

namespace math_utils {
  
  double pointToLineDistance(const cv::Point2f& point, 
                            const cv::Point2f& lineStart, 
                            const cv::Point2f& lineEnd) {
    double A = point.x - lineStart.x;
    double B = point.y - lineStart.y;
    double C = lineEnd.x - lineStart.x;
    double D = lineEnd.y - lineStart.y;
    
    double dot = A * C + B * D;
    double lenSq = C * C + D * D;
    double param = (lenSq != 0) ? dot / lenSq : -1;
    
    double xx, yy;
    if (param < 0) {
      xx = lineStart.x;
      yy = lineStart.y;
    } else if (param > 1) {
      xx = lineEnd.x;
      yy = lineEnd.y;
    } else {
      xx = lineStart.x + param * C;
      yy = lineStart.y + param * D;
    }
    
    double dx = point.x - xx;
    double dy = point.y - yy;
    return std::sqrt(dx * dx + dy * dy);
  }
  
  double polygonArea(const std::vector<cv::Point2f>& points) {
    if (points.size() < 3) return 0.0;
    
    double area = 0.0;
    int n = points.size();
    
    for (int i = 0; i < n; i++) {
      int j = (i + 1) % n;
      area += points[i].x * points[j].y;
      area -= points[j].x * points[i].y;
    }
    
    return std::abs(area) / 2.0;
  }
  
  double angleBetweenVectors(const cv::Point2f& v1, const cv::Point2f& v2) {
    double dot = v1.x * v2.x + v1.y * v2.y;
    double det = v1.x * v2.y - v1.y * v2.x;
    return std::atan2(det, dot);
  }
  
  cv::Point3f imageToWorld(const cv::Point2f& imagePoint,
                          const cv::Mat& cameraMatrix,
                          const cv::Mat& rvec,
                          const cv::Mat& tvec,
                          double Z) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    
    cv::Mat invR = R.t();
    cv::Mat invT = -invR * tvec;
    
    cv::Mat invK = cameraMatrix.inv();
    cv::Mat imagePointHomogeneous = (cv::Mat_<double>(3,1) << imagePoint.x, imagePoint.y, 1);
    cv::Mat worldPointHomogeneous = invR * invK * imagePointHomogeneous;
    
    double s = (Z + invT.at<double>(2)) / worldPointHomogeneous.at<double>(2);
    cv::Mat worldPoint = invT + s * worldPointHomogeneous;
    
    return cv::Point3f(worldPoint.at<double>(0), worldPoint.at<double>(1), worldPoint.at<double>(2));
  }
  
} // namespace math_utils