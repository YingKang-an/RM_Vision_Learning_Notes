/** 
 * @file GxIAPI.cpp
 * @brief 大恒相机API封装实现
 * 
 * @author \b  YinKang'an
 * @date       2026-01-26
 * @version    1.0.0
 * 
 * @copyright  MIT License 
 */

#include "GxIAPI.hpp"
#include <iostream>
#include <cstring>

// 相机配置默认值
CameraConfig::CameraConfig(): width(1920), height(1080), framerate(30.0), pixelFormat(GX_PIXEL_FORMAT_BAYER_BG8) 
{
}

GxCamera::GxCamera(int bufferCount) 
    : m_device(nullptr), m_isOpen(false), m_bufferCount(bufferCount),
      m_config(new CameraConfig()), m_buffers(nullptr),
      m_exposure(10000), m_gain(0), m_wbR(1.0), m_wbG(1.0), m_wbB(1.0)
{
    /** 
     * @brief 初始化相机对象
     * 
     * 设置默认参数并分配配置内存。
     * 
     * @note 此时相机尚未连接
     */
}

GxCamera::~GxCamera() {
    /**
     * @brief 析构函数
     * 
     * 确保相机正确关闭并释放所有资源。
     */
    close();
    delete m_config;
}

bool GxCamera::open(int deviceIndex) {
    /**
     * @brief 打开相机设备
     * 
     * 执行以下步骤：
     * 1. 初始化GX SDK
     * 2. 枚举可用设备
     * 3. 打开指定设备
     * 4. 初始化参数
     * 5. 配置采集缓冲区
     * 
     * @return 成功返回true，失败返回false并输出错误信息
     */
    
    // 检查是否已打开
    if (m_isOpen) {
        std::cerr << "相机已打开" << std::endl;
        return false;
    }
    
    // 1. 初始化SDK
    GX_STATUS status = GXInitLib();
    if (!checkStatus(status, "初始化SDK")) {
        return false;
    }
    
    // 2. 枚举设备
    uint32_t deviceNum = 0;
    status = GXUpdateAllDeviceList(&deviceNum, 1000);
    if (!checkStatus(status, "枚举设备") || deviceNum == 0) {
        std::cerr << "未找到相机设备" << std::endl;
        GXCloseLib();
        return false;
    }
    
    // 3. 打开设备
    GX_OPEN_PARAM openParam;
    openParam.accessMode = GX_ACCESS_EXCLUSIVE;
    openParam.openMode = GX_OPEN_INDEX;
    char indexStr[10];
    snprintf(indexStr, sizeof(indexStr), "%d", deviceIndex);
    openParam.pszContent = indexStr;
    
    status = GXOpenDevice(&openParam, &m_device);
    if (!checkStatus(status, "打开设备")) {
        GXCloseLib();
        return false;
    }
    
    // 4. 初始化参数
    // 关闭所有自动模式
    GXSetEnum(m_device, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetEnum(m_device, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetEnum(m_device, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    
    // 读取当前参数
    GXGetFloat(m_device, GX_FLOAT_EXPOSURE_TIME, &m_exposure);
    GXGetFloat(m_device, GX_FLOAT_GAIN, &m_gain);
    
    // 5. 配置缓冲区
    if (!initBuffers(m_bufferCount)) {
        GXCloseDevice(m_device);
        GXCloseLib();
        return false;
    }
    
    // 启动采集
    status = GXStreamOn(m_device);
    if (!checkStatus(status, "启动采集")) {
        releaseBuffers();
        GXCloseDevice(m_device);
        GXCloseLib();
        return false;
    }
    
    m_isOpen = true;
    std::cout << "相机打开成功，设备索引: " << deviceIndex << std::endl;
    return true;
}

void GxCamera::close() {
    /**
     * @brief 关闭相机设备
     * 
     * 按正确顺序释放所有资源，确保没有内存泄漏。
     */
    
    if (!m_isOpen) return;
    
    // 停止采集
    GXStreamOff(m_device);
    
    // 释放缓冲区
    releaseBuffers();
    
    // 关闭设备
    GXCloseDevice(m_device);
    m_device = nullptr;
    
    // 关闭SDK
    GXCloseLib();
    
    m_isOpen = false;
    std::cout << "相机已关闭" << std::endl;
}

bool GxCamera::setExposure(double exposureTime) {
    /**
     * @brief 设置曝光时间
     * 
     * @param exposureTime 曝光时间（微秒）
     * @return 设置成功返回true
     */
    
    if (!m_isOpen) return false;
    
    GXSetEnum(m_device, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GX_STATUS status = GXSetFloat(m_device, GX_FLOAT_EXPOSURE_TIME, exposureTime);
    
    if (checkStatus(status, "设置曝光")) {
        m_exposure = exposureTime;
        return true;
    }
    return false;
}

// 其他函数实现类似...