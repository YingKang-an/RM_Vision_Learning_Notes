#include <opencv2/opencv.hpp>
#include "GxIAPI.h"
#include <iostream>

// 相机句柄
GX_DEV_HANDLE hDevice = nullptr;
// 滑动条初始值
int g_exposureVal = 10000;  // 曝光(μs)
int g_gainVal = 0;          // 增益(×10)
int g_wbBVal = 100;         // 蓝通道白平衡
int g_wbGVal = 100;         // 绿通道白平衡
int g_wbRVal = 100;         // 红通道白平衡

// 回调函数
// 曝光调节
void onExposureTrackbar(int val, void*) {
    if (hDevice == nullptr || val < 0) return;
    double exposure = (double)val;
    GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GX_STATUS status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
    if (status == GX_STATUS_SUCCESS) g_exposureVal = val;
}
// 增益调节
void onGainTrackbar(int val, void*) {
    if (hDevice == nullptr || val < 0) return;
    double gain = (double)val / 10.0;
    GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GX_STATUS status = GXSetFloat(hDevice, GX_FLOAT_GAIN, gain);
    if (status == GX_STATUS_SUCCESS) g_gainVal = val;
}
// 蓝通道白平衡调节
void onWB_B_Trackbar(int val, void*) {
    if (hDevice == nullptr || val < 0) return;
    double wbRatio = (double)val / 100.0;
    GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    GX_STATUS status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, wbRatio);
    if (status == GX_STATUS_SUCCESS) g_wbBVal = val;
}
// 绿通道白平衡调节
void onWB_G_Trackbar(int val, void*) {
    if (hDevice == nullptr || val < 0) return;
    double wbRatio = (double)val / 100.0;
    GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    GX_STATUS status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, wbRatio);
    if (status == GX_STATUS_SUCCESS) g_wbGVal = val;
}
// 红通道白平衡调节
void onWB_R_Trackbar(int val, void*) {
    if (hDevice == nullptr || val < 0) return;
    double wbRatio = (double)val / 100.0;
    GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    GX_STATUS status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, wbRatio);
    if (status == GX_STATUS_SUCCESS) g_wbRVal = val;
}

int main(int argc, char* argv[])
{
    // 1.初始化
    GX_STATUS emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS) return -1;
    // 2.枚举设备
    uint32_t DeviceNum = 0;
    emStatus = GXUpdateAllDeviceList(&DeviceNum, 1000);
    if (DeviceNum <= 0) { GXCloseLib();return -1;}
    // 3.打开设备
    hDevice = nullptr;
    GX_OPEN_PARAM stOpenParam;
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    char szContent[] = "1";
    stOpenParam.pszContent = szContent;
    emStatus = GXOpenDevice(&stOpenParam, &hDevice);
    if (emStatus != GX_STATUS_SUCCESS) { GXCloseLib();return -1;}
    // 4.初始化相机参数 关闭自动模式
    GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    // 5.读取初始参数并同步到滑动条
    double initExposure, initGain, initWB_B, initWB_G, initWB_R;
    GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &initExposure);
    GXGetFloat(hDevice, GX_FLOAT_GAIN, &initGain);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
    GXGetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, &initWB_B);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
    GXGetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, &initWB_G);
    GXSetEnum(hDevice, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
    GXGetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, &initWB_R);
    g_exposureVal = (int)initExposure;
    g_gainVal = (int)(initGain * 10);
    g_wbBVal = (int)(initWB_B * 100);
    g_wbGVal = (int)(initWB_G * 100);
    g_wbRVal = (int)(initWB_R * 100);
    // 6.配置采集Buffer
    const int BUF_COUNT = 5;
    GX_FRAME_BUFFER frameBufferArray[BUF_COUNT];
    GX_FRAME_BUFFER* pFrameBufferPtrs[BUF_COUNT];
    for (int i = 0; i < BUF_COUNT; i++) pFrameBufferPtrs[i] = &frameBufferArray[i];
    GXSetAcqusitionBufferNumber(hDevice, BUF_COUNT);
    GXStreamOn(hDevice);
    // 7.创建窗口+滑动条
    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
    cv::createTrackbar("曝光",      "Camera", &g_exposureVal, 100000, onExposureTrackbar);
    cv::createTrackbar("增益",      "Camera", &g_gainVal,     100,   onGainTrackbar);
    cv::createTrackbar("白平衡-蓝", "Camera", &g_wbBVal,      300,    onWB_B_Trackbar);
    cv::createTrackbar("白平衡-绿", "Camera", &g_wbGVal,      300,    onWB_G_Trackbar);
    cv::createTrackbar("白平衡-红", "Camera", &g_wbRVal,      300,    onWB_R_Trackbar);
    // 8.循环采集显示
    uint32_t actualFrameCount = 0;
    bool Quit = false;
    while (!Quit) {
        // 获取图像
        emStatus = GXDQAllBufs(hDevice, pFrameBufferPtrs, BUF_COUNT, &actualFrameCount, 1000);
        if (emStatus != GX_STATUS_SUCCESS) continue;
        if (actualFrameCount > 0) {
            GX_FRAME_BUFFER* pFrame = pFrameBufferPtrs[actualFrameCount - 1];
            if (pFrame->nStatus == GX_FRAME_STATUS_SUCCESS && pFrame->pImgBuf != nullptr) {
                cv::Mat img(pFrame->nHeight, pFrame->nWidth, CV_8UC1, pFrame->pImgBuf);
                cv::Mat imgShow;
                cv::cvtColor(img, imgShow, cv::COLOR_BayerBG2BGR);
                cv::imshow("Camera", imgShow);
            }
        }
        // 归还Buffer
        GXQAllBufs(hDevice);
        // 按键检测
        int key = cv::waitKey(1);
        if (key == 27) Quit = true;
    }
    // 9. 释放资源
    GXStreamOff(hDevice);
    GXCloseDevice(hDevice);
    GXCloseLib();
    cv::destroyAllWindows();
}