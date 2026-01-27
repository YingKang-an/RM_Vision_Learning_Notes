/**
 * @file       GxIAPI.hpp
 * 
 * @brief      GXIAPI 封装头文件
 *             该文件定义了大恒相机的接口
 * 
 * @author @b  YinKang'an
 * @date   @b  2026-01-26
 * @version    1.0.0
 * 
 * @copyright  MIT License
 * 
 * @note       需要官网下载GxIAPI_SDK 并配置环境变量
 * 
 * @see        GxIAPI官方文档
 * @link   @b  https://www.daheng-image.com
 */

#ifndef GxIAPI_HPP
#define GxIAPI_HPP

#include <opencv2/opencv.hpp>
#include "GxIAPI.h"

/**
 * @defgroup   CameraControl 相机控制模块
 * @brief      大恒工业相机控制相关类和函数
 * 
 * @brief      该模块封装了大恒相机的初始化、参数设置、图像采集等功能，
 *             提供简单易用的C++接口
 */

// 前向声明
struct CameraConfig;

/**
 * @brief 相机控制器类
 * @ingroup CameraControl
 * 
 * 提供完整的大恒相机控制功能，包括：
 * - 设备发现与连接
 * - 参数调节（曝光、增益、白平衡）
 * - 图像采集与处理
 * - 资源管理
 * 
 * @code{.cpp}
 * // 使用示例
 * GxCamera camera;
 * if (camera.open(1)) {
 *     camera.setExposure(10000);
 *     cv::Mat frame = camera.capture();
 * }
 * @endcode
 */
class GxCamera {
public:
    /**
     * @brief 默认构造函数
     * @param bufferCount 采集缓冲区数量，默认5个
     * 
     * @note 缓冲区数量影响内存占用和采集性能
     */
    GxCamera(int bufferCount = 5);
    
    /**
     * @brief 析构函数
     * @warning 会自动关闭相机并释放资源
     */
    ~GxCamera();
    
    /**
     * @brief 打开指定索引的相机
     * 
     * @param deviceIndex 相机设备索引（从1开始）
     * @return bool 成功打开返回true，否则false
     * 
     * @note 默认以独占模式打开相机
     * @warning 同时只能有一个程序访问相机
     * 
     * @code{.cpp}
     * GxCamera cam;
     * if (!cam.open(1)) {
     *     std::cerr << "无法打开相机" << std::endl;
     *     return;
     * }
     * @endcode
     */
    bool open(int deviceIndex = 1);
    
    /**
     * @brief 关闭相机并释放资源
     * 
     * 按正确顺序关闭相机：
     * 1. 停止采集
     * 2. 关闭设备
     * 3. 关闭SDK库
     */
    void close();
    
    /**
     * @brief 设置曝光时间
     * 
     * @param exposureTime 曝光时间（微秒）
     * @return bool 设置成功返回true
     * 
     * @note 自动关闭自动曝光模式
     * @warning 过长的曝光时间可能导致帧率下降
     */
    bool setExposure(double exposureTime);
    
    /**
     * @brief 设置增益
     * 
     * @param gain 增益倍数（0-100.0）
     * @return bool 设置成功返回true
     * 
     * @note 实际增益 = gain × 10存储
     */
    bool setGain(double gain);
    
    /**
     * @brief 设置白平衡系数
     * 
     * @param channel 颜色通道：'R','G','B'
     * @param ratio 白平衡系数（0.0-3.0）
     * @return bool 设置成功返回true
     * 
     * @note 默认系数为1.0，绿通道通常作为基准
     */
    bool setWhiteBalance(char channel, double ratio);
    
    /**
     * @brief 采集一帧图像
     * 
     * @param timeout 超时时间（毫秒），默认1000ms
     * @return cv::Mat 采集到的图像，失败返回空矩阵
     * 
     * @note 图像格式为BayerBG，需要转换为BGR显示
     * @warning 需要先调用open()打开相机
     */
    cv::Mat capture(int timeout = 1000);
    
    /**
     * @brief 获取当前曝光时间
     * @return double 当前曝光时间（微秒）
     */
    double getExposure() const;
    
    /**
     * @brief 获取当前增益
     * @return double 当前增益倍数
     */
    double getGain() const;
    
    /**
     * @brief 获取指定通道的白平衡系数
     * @param channel 颜色通道：'R','G','B'
     * @return double 白平衡系数
     */
    double getWhiteBalance(char channel) const;
    
    /**
     * @brief 检查相机是否已打开
     * @return bool 相机打开状态
     */
    bool isOpen() const;

private:
    /**
     * @brief 初始化采集缓冲区
     * @param count 缓冲区数量
     * @return bool 初始化成功返回true
     */
    bool initBuffers(int count);
    
    /**
     * @brief 释放采集缓冲区
     */
    void releaseBuffers();
    
    /**
     * @brief 内部错误处理函数
     * @param status GX API状态码
     * @param operation 操作描述
     * @return bool 操作成功返回true
     */
    bool checkStatus(GX_STATUS status, const std::string& operation);
    
    GX_DEV_HANDLE m_device;            /**< 相机设备句柄 */
    bool m_isOpen;                     /**< 相机打开状态 */
    int m_bufferCount;                 /**< 缓冲区数量 */
    CameraConfig* m_config;            /**< 相机配置参数 */
    GX_FRAME_BUFFER* m_buffers;        /**< 采集缓冲区数组 */
    double m_exposure;                 /**< 当前曝光时间 */
    double m_gain;                     /**< 当前增益 */
    double m_wbR, m_wbG, m_wbB;        /**< 白平衡系数 */
};

/**
 * @brief 相机配置结构体
 * @ingroup CameraControl
 * 
 * 存储相机的配置参数，用于初始化相机设置。
 */
struct CameraConfig {
    int width;           /**< 图像宽度 */
    int height;          /**< 图像高度 */
    double framerate;    /**< 帧率 */
    int pixelFormat;     /**< 像素格式 */
    
    /**
     * @brief 默认构造函数
     * 设置默认参数值
     */
    CameraConfig();
};

/**
 * @brief 全局相机参数回调函数
 * @ingroup CameraControl
 * 
 * 这些函数用于OpenCV滑动条回调，调节相机参数。
 */

/**
 * @brief 曝光调节回调
 * @param val 滑动条值（微秒）
 * @param userdata 用户数据（GxCamera指针）
 */
void onExposureChange(int val, void* userdata);

/**
 * @brief 增益调节回调
 * @param val 滑动条值（×10）
 * @param userdata 用户数据（GxCamera指针）
 */
void onGainChange(int val, void* userdata);

/**
 * @brief 白平衡调节回调
 * @param val 滑动条值（×100）
 * @param userdata 用户数据（GxCamera指针）
 */
void onWhiteBalanceChange(int val, void* userdata);

#endif // GXIAPI_HPP