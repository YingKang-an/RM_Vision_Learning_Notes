/**
 * @author     YinKang'an
 * @date       2026-02-01
 * @file       image_processor.hpp
 * @brief      图像处理线程管理头文件
 */

#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <atomic>
#include <mutex>
#include "thread_pool.hpp"

// 图像处理结果结构体
struct ProcessResult {
  bool chessboardFound = false;
  std::vector<cv::Point2f> corners;
  cv::Mat processedImage;
  cv::Mat cornerImage;  // 新增：专门用于角点显示的图像
  int imageIndex = -1;
};

class ImageProcessor {
public:
  // 构造函数
  ImageProcessor(size_t numThreads = 12);
  
  // 析构函数
  ~ImageProcessor();
  
  // 处理图像（异步）
  std::future<ProcessResult> processImageAsync(const cv::Mat& image, int imageIndex);
  
  // 获取棋盘格检测结果
  bool detectChessboard(const cv::Mat& image, std::vector<cv::Point2f>& corners);
  
  // 绘制处理结果
  cv::Mat drawResult(const cv::Mat& image, const ProcessResult& result, int capturedCount);
  
  // 获取线程池大小
  size_t getThreadCount() const { return threadPool.size(); }

private:
  ThreadPool threadPool;                    // 线程池
  cv::Size boardSize;                       // 棋盘格尺寸
  float squareSize;                         // 方格大小
  std::atomic<int> processedCount;          // 已处理图像计数
};

#endif // IMAGE_PROCESSOR_HPP