/**
 * @author \b     YinKang'an
 * @date          2026-01-30
 * @version       V2.0.0
 * @brief         大恒相机驱动
 * @note \b       [优化]
 *                该版本适配最新版本相机SDK
 *                消除OpenCV(4.6.0)滑动条警告
 *                优化滑动条最大值，充分释放相机性能极限
 * 
 * @attention \b  [注意]
 *                相机版本: MER-139-210U3C
 *                OpenCV版本: 4.6.0
 */

#include <opencv2/opencv.hpp>
#include "GxIAPI.h"
#include <iostream>

// 相机句柄
GX_DEV_HANDLE hDevice = nullptr;
// 滑动条初始值
typedef struct CameraParams {
int ExposureVal = 10000;                                        // 曝光(μs)
int GainVal = 16000;            /**< Min:0.0000 Max:16.000 */   //增益
int bWhiteBal = 79961;          /**< Min:1.0000 Max:7.9961 */   //蓝通道白平衡
int gWhiteBal = 79961;          /**< Min:1.0000 Max:7.9961 */   //绿通道白平衡
int rWhiteBal = 79961;          /**< Min:1.0000 Max:7.9961 */   //红通道白平衡
}CameraParams;                  /**< 参数在 /Galaxy_camera/bin/GalaxyView 页面右下角体现 */

// 曝光调节
void onExposureTrackbar(int val, void* userdata) {
  if (hDevice == nullptr || val < 0) return;
  double exposure = (double)val * 1.0;          /**< 曝光时间单位为微秒 */
  GX_STATUS status = GXSetEnumValueByString(hDevice, "ExposureAuto", "Off");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetFloatValue(hDevice, "ExposureTime", exposure);
  if (status == GX_STATUS_SUCCESS) {
    int* pVal = (int*)userdata;
    if (pVal != nullptr) *pVal = val;
  }
}

// 增益调节
void onGainTrackbar(int val, void* userdata) {
  if (hDevice == nullptr || val < 0) return;
  double gain = (double)val / 1000.0;          /**< 16000/[1000.0] = 16.0000 */
  GX_STATUS status = GXSetEnumValueByString(hDevice, "GainAuto", "Off");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetFloatValue(hDevice, "Gain", gain);
  if (status == GX_STATUS_SUCCESS) {
    int* pVal = (int*)userdata;
    if (pVal != nullptr) *pVal = val;
  }
}

// 蓝通道白平衡调节
void onWB_B_Trackbar(int val, void* userdata) {
  if (hDevice == nullptr || val < 0) return;
  double wbRatio = (double)val / 10000.0;          /**< 79961/[10000.0] = 7.9961 */
  GX_STATUS status = GXSetEnumValueByString(hDevice, "BalanceWhiteAuto", "Off");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetEnumValueByString(hDevice, "BalanceRatioSelector", "Blue");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetFloatValue(hDevice, "BalanceRatio", wbRatio);
  if (status == GX_STATUS_SUCCESS) {
    int* pVal = (int*)userdata;
    if (pVal != nullptr) *pVal = val;
  }
}

// 绿通道白平衡调节
void onWB_G_Trackbar(int val, void* userdata) {
  if (hDevice == nullptr || val < 0) return;
  double wbRatio = (double)val / 10000.0;          /**< 79961/[10000.0] = 7.9961 */
  GX_STATUS status = GXSetEnumValueByString(hDevice, "BalanceWhiteAuto", "Off");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetEnumValueByString(hDevice, "BalanceRatioSelector", "Green");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetFloatValue(hDevice, "BalanceRatio", wbRatio);
  if (status == GX_STATUS_SUCCESS) {
    int* pVal = (int*)userdata;
    if (pVal != nullptr) *pVal = val;
  }
}

// 红通道白平衡调节
void onWB_R_Trackbar(int val, void* userdata) {
  if (hDevice == nullptr || val < 0) return;
  double wbRatio = (double)val / 10000.0;          /**< 79961/[10000.0] = 7.9961 */
  GX_STATUS status = GXSetEnumValueByString(hDevice, "BalanceWhiteAuto", "Off");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetEnumValueByString(hDevice, "BalanceRatioSelector", "Red");
  if (status != GX_STATUS_SUCCESS) return;
  status = GXSetFloatValue(hDevice, "BalanceRatio", wbRatio);
  if (status == GX_STATUS_SUCCESS) {
    int* pVal = (int*)userdata;
    if (pVal != nullptr) *pVal = val;
  }
}


int main(int argc, char* argv[]) {
  // 初始化SDK
  GX_STATUS status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "SDK初始化失败，错误码：" << status << std::endl;
    return -1;
  }

  // 枚举设备
  uint32_t DeviceNum = 0;
  status = GXUpdateAllDeviceList(&DeviceNum, 1000);
  if (DeviceNum <= 0) {
    std::cerr << "未检测到相机设备" << std::endl;
    GXCloseLib();              /**< 退出前关闭SDK */
    return -1;
  }
  std::cout << "检测到 " << DeviceNum << " 台相机设备" << std::endl;

  // 打开设备
  hDevice = nullptr;
  GX_OPEN_PARAM stOpenParam;
  stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
  stOpenParam.openMode = GX_OPEN_INDEX;
  char szContent[] = "1";    // 打开第1台相机
  stOpenParam.pszContent = szContent;
  
  status = GXOpenDevice(&stOpenParam, &hDevice);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "打开相机失败，错误码：" << status << std::endl;
    GXCloseLib();
    return -1;
  }

  CameraParams params;
  // 初始化相机参数 - 关闭自动模式
  status = GXSetEnumValueByString(hDevice, "ExposureAuto", "Off");  /**<  关闭自动曝光 */
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "关闭自动曝光失败，错误码：" << status << std::endl;
  }
  status = GXSetEnumValueByString(hDevice, "GainAuto", "Off");       /**<  关闭自动增益 */
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "关闭自动增益失败，错误码：" << status << std::endl;
  }
  status = GXSetEnumValueByString(hDevice, "BalanceWhiteAuto", "Off");/**<  关闭自动白平衡 */
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "关闭自动白平衡失败，错误码：" << status << std::endl;
  }

  // 读取初始参数并同步到滑动条
  double initExposure = 10000.0, initGain = 0.0, initWB_B = 1.0, initWB_G = 1.0, initWB_R = 1.0;
  GX_FLOAT_VALUE FloatValue;

  // 获取初始曝光时间
  status = GXGetFloatValue(hDevice, "ExposureTime", &FloatValue);
  if (status == GX_STATUS_SUCCESS) {
    initExposure = FloatValue.dCurValue * 1.0;
  } else {
    std::cerr << "获取初始曝光时间失败，错误码：" << status << "，使用默认值" << std::endl;
  }

  // 获取初始增益
  status = GXGetFloatValue(hDevice, "Gain", &FloatValue);
  if (status == GX_STATUS_SUCCESS) {
    initGain = FloatValue.dCurValue * 1000.0;
  } else {
    std::cerr << "获取初始增益失败，错误码：" << status << "，使用默认值" << std::endl;
  }

  // 获取蓝通道白平衡初始值
  status = GXSetEnumValueByString(hDevice, "BalanceRatioSelector", "Blue");
  if (status == GX_STATUS_SUCCESS) {
    status = GXGetFloatValue(hDevice, "BalanceRatio", &FloatValue);
    if (status == GX_STATUS_SUCCESS) {
      initWB_B = FloatValue.dCurValue * 10000.0;
    } else {
      std::cerr << "读取蓝通道白平衡失败，错误码：" << status << "，使用默认值" << std::endl;
    }
  } else {
      std::cerr << "选择蓝通道失败，错误码：" << status << "，使用默认值" << std::endl;
  }

  // 获取绿通道白平衡初始值
  status = GXSetEnumValueByString(hDevice, "BalanceRatioSelector", "Green");
  if (status == GX_STATUS_SUCCESS) {
      status = GXGetFloatValue(hDevice, "BalanceRatio", &FloatValue);
      if (status == GX_STATUS_SUCCESS) {
          initWB_G = FloatValue.dCurValue * 10000.0;
      } else {
          std::cerr << "读取绿通道白平衡失败，错误码：" << status << "，使用默认值" << std::endl;
      }
  } else {
      std::cerr << "选择绿通道失败，错误码：" << status << "，使用默认值" << std::endl;
  }

  // 获取红通道白平衡初始值
  status = GXSetEnumValueByString(hDevice, "BalanceRatioSelector", "Red");
  if (status == GX_STATUS_SUCCESS) {
    status = GXGetFloatValue(hDevice, "BalanceRatio", &FloatValue);
    if (status == GX_STATUS_SUCCESS) {
      initWB_R = FloatValue.dCurValue * 10000.0;
    } else {
      std::cerr << "读取红通道白平衡失败，错误码：" << status << "，使用默认值" << std::endl;
    }
  } else {
    std::cerr << "选择红通道失败，错误码：" << status << "，使用默认值" << std::endl;
  }

  // 同步初始值到滑动条变量
  params.ExposureVal = (int)initExposure;
  params.GainVal = (int)(initGain);  
  params.bWhiteBal = (int)(initWB_B);
  params.gWhiteBal = (int)(initWB_G);
  params.rWhiteBal = (int)(initWB_R);

  // 配置采集Buffer
  const int BUF_COUNT = 5;
  GX_FRAME_BUFFER frameBufferArray[BUF_COUNT];
  GX_FRAME_BUFFER* pFrameBufferPtrs[BUF_COUNT];
  for (int i = 0; i < BUF_COUNT; i++) {
    pFrameBufferPtrs[i] = &frameBufferArray[i];
  }
  status = GXSetAcqusitionBufferNumber(hDevice, BUF_COUNT);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "配置采集Buffer失败，错误码：" << status << "，使用默认缓冲区" << std::endl;
  }

  // 启动采集线程
  status = GXStreamOn(hDevice);
  if (status != GX_STATUS_SUCCESS) {
    std::cerr << "启动采集失败，错误码：" << status << std::endl;
    GXCloseDevice(hDevice);
    GXCloseLib();
    return -1;
  }

  // 创建窗口,滑动条
  cv::namedWindow("Camera", cv::WINDOW_NORMAL);
  cv::waitKey(10);  // 新增：等待窗口初始化完成，避免NULL句柄

  cv::createTrackbar("曝光[μs](Min:1.0000 Max:100000)", "Camera", NULL, 100000, onExposureTrackbar, &(params.ExposureVal));
  // 增益滑动条
  cv::createTrackbar("增益大小(Min:0.0000 Max:16.0000)", "Camera", NULL, 16000, onGainTrackbar, &(params.GainVal));
  // 白平衡滑动条
  cv::createTrackbar("白平衡-蓝(Min:1.0000 Max:7.9961)", "Camera", NULL, 79961, onWB_B_Trackbar, &(params.bWhiteBal));
  cv::createTrackbar("白平衡-绿(Min:1.0000 Max:7.9961)", "Camera", NULL, 79961, onWB_G_Trackbar, &(params.gWhiteBal));
  cv::createTrackbar("白平衡-红(Min:1.0000 Max:7.9961)", "Camera", NULL, 79961, onWB_R_Trackbar, &(params.rWhiteBal));

  // 设置滑动条初始值
  cv::setTrackbarPos("曝光[μs](Min:1.0000 Max:100000)", "Camera", params.ExposureVal);
  cv::setTrackbarPos("增益大小(Min:0.0000 Max:16.0000)", "Camera", params.GainVal);
  cv::setTrackbarPos("白平衡-蓝(Min:1.0000 Max:7.9961)", "Camera", params.bWhiteBal);
  cv::setTrackbarPos("白平衡-绿(Min:1.0000 Max:7.9961)", "Camera", params.gWhiteBal);
  cv::setTrackbarPos("白平衡-红(Min:1.0000 Max:7.9961)", "Camera", params.rWhiteBal);

  // 循环采集显示
  uint32_t actualFrameCount = 0;
  bool Quit = false;
  std::cout << "开始采集，按ESC键退出..." << std::endl;

  while (!Quit) {
    // 获取图像Buffer
    status = GXDQAllBufs(hDevice, pFrameBufferPtrs, BUF_COUNT, &actualFrameCount, 1000);
    if (status != GX_STATUS_SUCCESS) {
      continue;
    }

    // 处理有效图像
    if (actualFrameCount > 0) {
      GX_FRAME_BUFFER* pFrame = pFrameBufferPtrs[actualFrameCount - 1];  /**< ppFrameBufferArray[nFrameCount - 1]存储最新的图像 */
      if (pFrame->nStatus == GX_FRAME_STATUS_SUCCESS && pFrame->pImgBuf != nullptr) {
        // 转换BayerBG格式为RGB
        cv::Mat img(pFrame->nHeight, pFrame->nWidth, CV_8UC1, pFrame->pImgBuf);
        cv::Mat imgShow;
        cv::cvtColor(img, imgShow, cv::COLOR_BayerBG2BGR);
        cv::imshow("Camera", imgShow);
      }
    }

    // 归还Buffer
    GXQAllBufs(hDevice);

    // 按键检测（ESC退出）
    int key = cv::waitKey(1);
    if (key == 27) {
      Quit = true;
    }
  }

  // 9. 释放资源
  GXStreamOff(hDevice);          // 停止采集
  GXCloseDevice(hDevice);        // 关闭设备
  GXCloseLib();                  // 释放SDK资源
  cv::destroyAllWindows();       // 销毁窗口

  std::cout << "程序正常退出" << std::endl;
  return 0;
}
