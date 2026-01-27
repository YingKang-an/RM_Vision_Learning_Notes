/**
 * @file       opencv.hpp
 * @brief      OpenCV图像处理功能头文件
 *             提供图像显示、参数调节界面、图像处理等OpenCV相关功能
 * 
 * @author \b  YinKang'an
 * @date       2026-01-26
 * @version    1.0.0
 * 
 * @copyright  MIT License   
 */

#ifndef OPENCV_HPP
#define OPENCV_HPP

#include <opencv2/opencv.hpp>
#include <functional>

/**
 * @defgroup ImageProcessing 图像处理模块
 * @brief OpenCV图像处理相关功能
 */

/**
 * @brief 图像显示控制器
 * @ingroup ImageProcessing
 * 
 * 管理OpenCV窗口，显示图像和调节参数。
 */
class ImageViewer {
public:
    /**
     * @brief 构造函数
     * @param windowName 窗口名称
     */
    ImageViewer(const std::string& windowName);
    
    /**
     * @brief 显示图像
     * @param image 要显示的图像
     * @param waitMs 等待时间（毫秒）
     */
    void show(const cv::Mat& image, int waitMs = 1);
    
    /**
     * @brief 添加曝光调节滑动条
     * @param initialValue 初始值
     * @param maxValue 最大值
     * @param callback 回调函数
     */
    void addExposureTrackbar(int initialValue, int maxValue, 
                           std::function<void(int)> callback);
    
    /**
     * @brief 添加增益调节滑动条
     * @param initialValue 初始值
     * @param maxValue 最大值
     * @param callback 回调函数
     */
    void addGainTrackbar(int initialValue, int maxValue,
                       std::function<void(int)> callback);
    
    /**
     * @brief 添加白平衡调节滑动条
     * @param channel 通道名称
     * @param initialValue 初始值
     * @param maxValue 最大值
     * @param callback 回调函数
     */
    void addWhiteBalanceTrackbar(const std::string& channel,
                               int initialValue, int maxValue,
                               std::function<void(int)> callback);
    
    /**
     * @brief 检查是否按下退出键
     * @return 按下ESC返回true
     */
    bool shouldQuit() const;
    
private:
    std::string m_windowName;
};

/**
 * @brief 图像处理工具函数
 * @ingroup ImageProcessing
 */

/**
 * @brief Bayer格式转BGR
 * @param bayerImage Bayer格式图像
 * @return BGR格式图像
 */
cv::Mat bayerToBGR(const cv::Mat& bayerImage);

/**
 * @brief 在图像上叠加参数信息
 * @param image 输入图像
 * @param exposure 曝光时间
 * @param gain 增益
 * @param wbR 红通道白平衡
 * @param wbG 绿通道白平衡
 * @param wbB 蓝通道白平衡
 */
void overlayCameraInfo(cv::Mat& image, double exposure, double gain,
                      double wbR, double wbG, double wbB);

#endif // OPENCV_HPP