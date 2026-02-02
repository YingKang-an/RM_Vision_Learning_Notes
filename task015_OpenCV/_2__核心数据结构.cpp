/********************************************************************
 * @author @b    YinKang'an
 * @date         2026.01.29
 * @brief        核心数据结构
 * ------------------------------------------------------------------
 * @note   @b    [声明说明]
 * \b            本文展示都是基本类型,`方法和属性`在创建之后点一下就能看到.本文仅演示基本方法
 * \b            智能指针在OpenCV基本类型中少用,仅了解即可.
 *********************************************************************/

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;

// 自定义类用于演示智能指针
class ImageProcessor {
public:
  int id;
  std::string name;
  cv::Mat image;
  
  ImageProcessor(int _id, const std::string& _name, const cv::Size& size) : id(_id), name(_name), image(size, CV_8UC3) {
    std::cout << "创建 ImageProcessor: " << name << " (ID: " << id << ")" << std::endl;
  }
  
  ~ImageProcessor() {
    std::cout << "销毁 ImageProcessor: " << name << " (ID: " << id << ")" << std::endl;
  }
  
  void generateGradient() {
    // 创建渐变效果
    for (int y = 0; y < image.rows; y++) {
      for (int x = 0; x < image.cols; x++) {
        cv::Vec3b& pixel = image.at<cv::Vec3b>(y, x);
        pixel[0] = static_cast<uchar>(255 * x / image.cols);      // B
        pixel[1] = static_cast<uchar>(255 * y / image.rows);      // G
        pixel[2] = static_cast<uchar>(255 * (1.0 - x / (double)image.cols)); // R
      }
    }
  }
  
  void displayInfo() const {
    std::cout << "处理器: " << name << ", 图像尺寸: " << image.cols << "×" << image.rows << ", 通道: " << image.channels() << std::endl;
  }
};

int main(int argc, char* argv[]) {

  std::cout << "========== OpenCV 核心数据类型综合示例 ==========" << std::endl;

  // =========================================================================================================
  // 1. Mat 类 - 图像存储的核心
  // Mat class - Core of image storage
  // =========================================================================================================
  
  std::cout << "\n\n1. Mat 类演示:" << std::endl;
  std::cout << "==================" << std::endl;
  std::cout << std::endl;

  //----------------------------------------------------------------------------------------------------------
  // 1.1 创建不同类型和尺寸的 Mat
  cv::Mat gray_img(480, 640, CV_8UC1);            // 灰度图像/Grayscale image
  cv::Mat color_img(480, 640, CV_8UC3);           // 彩色图像/Color image
  cv::Mat float_img(100, 100, CV_32FC1);          // 浮点图像/Floating point image

  //----------------------------------------------------------------------------------------------------------
  // 1.2 使用不同方法初始化
  cv::Mat zeros_mat = cv::Mat::zeros(3, 3, CV_8UC1);
  cv::Mat ones_mat = cv::Mat::ones(3, 3, CV_32FC1);
  cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32FC1);
  cv::Mat blue_img(300, 400, CV_8UC3, cv::Scalar(255, 0, 0));
  
  std::cout << "创建灰度图像: " << gray_img.cols << "×" << gray_img.rows << std::endl;
  std::cout << "创建彩色图像: " << color_img.cols << "×" << color_img.rows << std::endl;
  std::cout << "零矩阵:\n" << zeros_mat << std::endl;
  std::cout << "单位矩阵:\n" << eye_mat << std::endl;
  std::cout << std::endl;

  //----------------------------------------------------------------------------------------------------------
  // 1.3 像素访问方法比较
  cv::Mat test_img(500, 500, CV_8UC3);
  
  // 方法1: at<>()
  auto start = std::chrono::high_resolution_clock::now();                              /**< 记录开始时间/Record start time */

  //生成100x100像素的彩色图并按照特定规则为每个像素的 BGR 三个通道赋值
  for (int i = 0; i < 500; i++) {
    for (int j = 0; j < 500; j++) {
      //蓝色通道值由行号i决定，绿色通道值由列号j决定，红色通道值由i+j决定
      test_img.at<cv::Vec3b>(i, j) = cv::Vec3b(i % 256, j % 256, (i + j) % 256);       /**< at<>为每个像素的 BGR 三个通道赋值/Assign BGR values to each pixel */
    }
  }

  auto end = std::chrono::high_resolution_clock::now();                                /**< 记录结束时间/Record end time */
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);  /**< 记录耗时/Record duration */
  std::cout << "at<>() 方法时间: " << duration.count() << " μs" << std::endl;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // 方法2: ptr<>()
  start = std::chrono::high_resolution_clock::now();                                   /**< 记录开始时间/Record start time */

  for (int i = 0; i < 500; i++) {
    // 跳过 OpenCV 的安全检查，直接拿到图像第 i 行的内存起始地址
    cv::Vec3b* p_row = test_img.ptr<cv::Vec3b>(i);                                     /**< 获取第i行的指针/Get pointer to row i */
    for (int j = 0; j < 500; j++) {
      // 直接通过指针 *(p_row + j) 或 p_row[j] 访问第i行第j列的像素
      *(p_row + j) = cv::Vec3b(i % 256, j % 256, (i + j) % 256);                       /**< ptr<>为每个像素的 BGR 三个通道赋值/Assign BGR values to each pixel */
    }
  }

  end = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);       /**< 记录耗时/Record duration */
  std::cout << "ptr<>() 方法时间: " << duration.count() << " μs" << std::endl;

  std::cout << std::endl;
  imshow("test_img", test_img);

  //----------------------------------------------------------------------------------------------------------
  // 1.4 ROI 操作
  cv::Mat image(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  cv::Mat roi = image(cv::Rect(100, 100, 200, 150));                                   /**< 提取ROI/Extract ROI */
  roi.setTo(cv::Scalar(255, 255, 0));                                                  /**< 设置为青色/Set to cyan */
  imshow("image_ROI", image);
  
  //----------------------------------------------------------------------------------------------------------
  // 1.5 矩阵运算
  // Mat_<T>是「强类型矩阵」，声明时就固定数据类型，访问元素直接定位内存，无类型检查开销，运算效率更高
  cv::Mat A = (cv::Mat_<float>(2, 3) << 1, 2, 3, 4, 5, 6);                             /**< Mat_矩阵/Mat_ Matrix */
  cv::Mat B = (cv::Mat_<float>(2, 3) << 6, 5, 4, 3, 2, 1);                             /**< Mat_矩阵/Mat_ Matrix */
  // A: [1,2,3]  B: [6,5,4]
  //    [4,5,6]     [3,2,1]
  cv::Mat C = A + B;
  cv::Mat D = A - B;
  std::cout << "矩阵加法:\n" << C << std::endl;
  std::cout << "矩阵减法:\n" << D << std::endl;
  
  // =========================================================================================================
  // 2. 基本类型 - Point、Size、Rect、Scalar
  // Basic types - Point, Size, Rect, Scalar
  // =========================================================================================================
  
  std::cout << "\n\n2. 几何和颜色类型演示:" << std::endl;
  std::cout << "==================================" << std::endl;
  std::cout << std::endl;
  
  //__________________________________________________________________________________________________________
  // 2.1 Point - 点
  cv::Point   pt1(100, 200);                                                           /**< Point是Point2i简写/Point is Point2i shorthand */
  cv::Point2f pt2(100.5f, 200.7f);
  cv::Point3i pt3(100, 200, 300);
  
  std::cout << "整数点: (" << pt1.x << ", " << pt1.y << ")" << std::endl;
  std::cout << "浮点数点: (" << pt2.x << ", " << pt2.y << ")" << std::endl;
  std::cout << "3D点: (" << pt3.x << ", " << pt3.y << ", " << pt3.z << ")" << std::endl;
  std::cout << std::endl;
  //----------------------------------------------------------------------------------------------------------
  // 点运算
  cv::Point p1(10, 20);
  cv::Point p2(30, 40);
  cv::Point sum = p1 + p2;
  cv::Point diff = p2 - p1;
  
  std::cout << "点加法: (" << p1.x << "," << p1.y << ") + (" << p2.x << "," << p2.y << ") = (" << sum.x << "," << sum.y << ")" << std::endl;
  std::cout << "点减法: (" << p2.x << "," << p2.y << ") - (" << p1.x << "," << p1.y << ") = (" << diff.x << "," << diff.y << ")" << std::endl;
  std::cout << std::endl;
  //----------------------------------------------------------------------------------------------------------
  //__________________________________________________________________________________________________________
  // 2.2 Size - 尺寸
  cv::Size size1(640, 480);
  cv::Size2f size2(640.5f, 480.5f);
  
  std::cout << "整数尺寸: " << size1.width << "×" << size1.height << std::endl;
  std::cout << "浮点数尺寸: " << size2.width << "×" << size2.height << std::endl;
  std::cout << "整数尺寸面积: " << size1.area() << " 像素" << std::endl;
  std::cout << std::endl;
  //__________________________________________________________________________________________________________
  // 2.3 Rect - 矩形
  cv::Rect rect1(100, 100, 200, 150);
  cv::Rect rect2(150, 150, 200, 150);
  
  std::cout << "矩形1: 位置(" << rect1.x << "," << rect1.y << "), 尺寸(" << rect1.width << "×" << rect1.height << ")" << std::endl;
  std::cout << "矩形2: 位置(" << rect2.x << "," << rect2.y << "), 尺寸(" << rect2.width << "×" << rect2.height << ")" << std::endl;
  std::cout << std::endl;
  // 矩形运算
  cv::Rect intersect = rect1 & rect2;  // 交集/Intersection
  cv::Rect union_rect = rect1 | rect2; // 并集/Union
  
  std::cout << "交集: 位置(" << intersect.x << "," << intersect.y << "), 尺寸(" << intersect.width << "×" << intersect.height << ")" << std::endl;
  std::cout << "并集: 位置(" << union_rect.x << "," << union_rect.y << "), 尺寸(" << union_rect.width << "×" << union_rect.height << ")" << std::endl;
  std::cout << std::endl;
  //__________________________________________________________________________________________________________
  // 2.4 Scalar - 颜色/标量
  cv::Scalar blue  (255, 0, 0);
  cv::Scalar red   (0, 0, 255);
  cv::Scalar white (255, 255, 255);
  cv::Scalar gray  (128);
  
  std::cout << "蓝色: B=" << (int)blue[0] << ", G=" << (int)blue[1] << ", R=" << (int)blue[2] << std::endl;
  std::cout << "红色: B=" << (int)red[0] << ", G=" << (int)red[1] << ", R=" << (int)red[2] << std::endl;
  std::cout << "灰色: B=" << (int)gray[0] << ", G=" << (int)gray[1] << ", R=" << (int)gray[2] << std::endl;
  
  // 颜色混合
  // Color mixing
  cv::Scalar mixed = blue * 0.7 + red * 0.3;
  std::cout << "颜色混合 (70% 蓝色 + 30% 红色): B=" << (int)mixed[0] << ", G=" << (int)mixed[1] << ", R=" << (int)mixed[2] << std::endl;
  
  // =========================================================================================================
  // 3. Vec 类型 - 多通道数据
  // Vec type - Multi-channel data
  // =========================================================================================================
  
  std::cout << "\n\n3. Vec 类型演示:" << std::endl;
  std::cout << "==================" << std::endl;
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 3.1 创建不同类型的向量
  cv::Vec3b pixel_bgr(100, 150, 200);                                            /**< BGR 像素/BGR pixel */
  cv::Vec3f vector_3d(1.5f, 2.5f, 3.5f);                                         /**< 3D 向量/3D vector */
  cv::Vec4b pixel_bgra(100, 150, 200, 255);                                      /**< BGRA 像素/BGRA pixel */
  
  std::cout << "BGR 像素: B=" << (int)pixel_bgr[0] << ", G=" << (int)pixel_bgr[1] << ", R=" << (int)pixel_bgr[2] << std::endl;
  std::cout << "3D 向量: (" << vector_3d[0] << ", " << vector_3d[1] << ", " << vector_3d[2] << ")" << std::endl;
  std::cout << "BGRA 像素: B=" << (int)pixel_bgra[0] << ", G=" << (int)pixel_bgra[1] << ", R=" << (int)pixel_bgra[2] << ", A=" << (int)pixel_bgra[3] << std::endl;
  std::cout << std::endl;

  //----------------------------------------------------------------------------------------------------------
  // 3.2 向量运算
  cv::Vec3f v1(1.0f, 2.0f, 3.0f);
  cv::Vec3f v2(4.0f, 5.0f, 6.0f);
  
  cv::Vec3f v_sum = v1 + v2;
  cv::Vec3f v_diff = v2 - v1;
  float dot_product = v1.dot(v2);
  cv::Vec3f cross_product = v1.cross(v2);
  
  std::cout << "向量加法: (" << v_sum[0] << ", " << v_sum[1] << ", " << v_sum[2] << ")" << std::endl;
  std::cout << "向量减法: (" << v_diff[0] << ", " << v_diff[1] << ", " << v_diff[2] << ")" << std::endl;
  std::cout << "点积: " << dot_product << std::endl;
  std::cout << "叉积: (" << cross_product[0] << ", " << cross_product[1] 
        << ", " << cross_product[2] << ")" << std::endl;
  
  // =========================================================================================================
  // 4. Range 类型 - 连续序列-左闭右开[x1,x2)
  // Range type - Continuous sequences
  // =========================================================================================================
  
  std::cout << "\n\n4. Range 类型演示:" << std::endl;
  std::cout << "====================" << std::endl;
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 4.1 创建范围
  cv::Range rows(10, 20);  // 包含索引10-19/Contains indices 10-19
  cv::Range cols(5, 15);   // 包含索引5-14/Contains indices 5-14
  cv::Range all_rows = cv::Range::all();  // 所有行/All rows

  std::cout << "行范围: [" << rows.start << ", " << rows.end << "), 大小: " << rows.size() << std::endl;
  std::cout << "列范围: [" << cols.start << ", " << cols.end << "), 大小: " << cols.size() << std::endl;
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 4.2 在矩阵中使用Range
  cv::Mat matrix(30, 30, CV_8UC1);
  cv::randu(matrix, 0, 255);
  
  cv::Mat submatrix = matrix(rows, cols);
  std::cout << "子矩阵大小: " << submatrix.rows << "×" << submatrix.cols << std::endl;
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 4.3 使用Range::all()选择所有行或列
  cv::Mat all_columns = matrix(all_rows, cols);
  std::cout << "所有行，部分列: " << all_columns.rows << "×" << all_columns.cols << std::endl;
  
  // =========================================================================================================
  // 5. Ptr 类型 - 智能指针
  // Ptr type - Smart pointers
  // =========================================================================================================
  
  std::cout << "\n\n5. Ptr 类型演示:" << std::endl;
  std::cout << "==================" << std::endl;
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 5.1 创建智能指针
  cv::Ptr<cv::Mat> smart_mat = cv::makePtr<cv::Mat>(100, 100, CV_8UC3);
  smart_mat->setTo(cv::Scalar(255, 0, 0));  // 设置为蓝色/Set to blue/(*smart_mat).setTo();
  
  std::cout << "智能指针管理的图像: " << smart_mat->cols << "×" << smart_mat->rows << ", 通道数: " << smart_mat->channels() << std::endl;
  std::cout << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 5.2 引用计数演示
  {
    cv::Ptr<cv::Mat> copy1 = smart_mat;  // 引用计数+1/Reference count +1
    cv::Ptr<cv::Mat> copy2 = smart_mat;  // 引用计数+1/Reference count +1
    
    std::cout << "创建了两个副本，当前引用计数: 3" << std::endl;
    
    // 离开作用域时，copy1和copy2自动销毁，引用计数减少
  }
  std::cout << "离开作用域后，引用计数恢复到: 1" << std::endl;
  
  //----------------------------------------------------------------------------------------------------------
  // 5.3 使用智能指针管理自定义对象
  cv::Ptr<ImageProcessor> processor = cv::makePtr<ImageProcessor>(1, "渐变生成器", cv::Size(400, 300));
  processor->generateGradient();
  processor->displayInfo();
  
  //----------------------------------------------------------------------------------------------------------
  // 5.4 智能指针在容器中的使用
  std::vector<cv::Ptr<ImageProcessor>> processors;
  
  for (int i = 0; i < 3; i++) {
    std::string name = "处理器_" + std::to_string(i + 1);
    cv::Ptr<ImageProcessor> proc = cv::makePtr<ImageProcessor>(i + 10, name, cv::Size(200 + i * 50, 150 + i * 50));
    proc->generateGradient();
    processors.push_back(proc);
  }
  
  std::cout << "处理器列表大小: " << processors.size() << std::endl;
  for (const auto& proc : processors) {
    proc->displayInfo();
  }
  















  // =========================================================================================================
  // 6. 综合应用示例
  // Comprehensive application example
  // =========================================================================================================
  
  std::cout << "\n\n6. 综合应用示例:" << std::endl;
  std::cout << "======================================\n" << std::endl;
  
  // 6.1 创建画布
  cv::Mat canvas(600, 800, CV_8UC3, cv::Scalar(240, 240, 240));
  
  // 6.2 使用Point绘制点
  std::vector<cv::Point> points;
  points.push_back(cv::Point(100, 100));
  points.push_back(cv::Point(200, 150));
  points.push_back(cv::Point(300, 120));
  points.push_back(cv::Point(400, 180));
  points.push_back(cv::Point(500, 100));
  
  for (const auto& pt : points) {
    cv::circle(canvas, pt, 5, cv::Scalar(0, 0, 255), -1);  // 绘制红色点/Draw red points
  }
  
  // 6.3 使用Rect绘制矩形
  cv::Rect rectangle1(50, 250, 200, 100);
  cv::Rect rectangle2(150, 300, 200, 100);
  
  cv::rectangle(canvas, rectangle1, cv::Scalar(255, 0, 0), 2);  // 蓝色边框/Blue border
  cv::rectangle(canvas, rectangle2, cv::Scalar(0, 255, 0), -1); // 绿色填充/Green fill
  
  // 绘制交集区域
  cv::Rect intersection = rectangle1 & rectangle2;
  if (intersection.width > 0 && intersection.height > 0) {
    cv::rectangle(canvas, intersection, cv::Scalar(0, 0, 255), 2);  // 红色边框/Red border
  }
  
  // 6.4 使用Size计算位置
  cv::Size text_size = cv::getTextSize("OpenCV Data Types", cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, nullptr);
  cv::Point text_origin((canvas.cols - text_size.width) / 2, 50);
  
  // 在图像上添加标题文字
  cv::putText(canvas, "OpenCV Data Types", text_origin, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
  
  // 6.5 使用Range选择区域并处理
  cv::Range roi_rows(400, 550);
  cv::Range roi_cols(100, 700);
  cv::Mat roi_region = canvas(roi_rows, roi_cols);
  
  // 创建渐变效果
  for (int y = 0; y < roi_region.rows; y++) {
    cv::Vec3b* row = roi_region.ptr<cv::Vec3b>(y);
    for (int x = 0; x < roi_region.cols; x++) {
      // 使用Vec3b处理像素
      // Process pixels using Vec3b
      cv::Vec3b& pixel = row[x];
      pixel[0] = static_cast<uchar>(255 * x / roi_region.cols);      // B
      pixel[1] = static_cast<uchar>(255 * y / roi_region.rows);      // G
      pixel[2] = static_cast<uchar>(255 * (1.0 - x / (double)roi_region.cols)); // R
    }
  }
  
  // 6.6 添加说明文本
  std::vector<std::string> legends = {
    "Red dots: Point type",
    "Blue rectangle: Rect type (border)",
    "Green rectangle: Rect type (filled)",
    "Red rectangle: Intersection of two rectangles",
    "Bottom gradient: Created using Range and Vec types"
  };
  
  for (size_t i = 0; i < legends.size(); i++) {
    cv::putText(canvas, legends[i], cv::Point(50, 200 + i * 30), 
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
  }
  
  // =========================================================================================================
  // 7. 显示和保存结果
  // Display and save results
  // =========================================================================================================
  
  std::cout << "\n显示综合示例结果" << std::endl;
  
  // 显示图像
  cv::imshow("OpenCV Core Data Types Demo", canvas);  // 主窗口/Main window
  
  // 显示智能指针管理的图像
  if (processor && !processor->image.empty()) {
    cv::imshow("Gradient Image (Smart Pointer)", processor->image);  // 渐变图像窗口/Gradient image window
  }
  
  // 保存图像
  cv::imwrite("opencv_core_types_demo.jpg", canvas);
  std::cout << "图像已保存为 opencv_core_types_demo.jpg" << std::endl;
  
  // 等待按键
  std::cout << "\n按任意键退出" << std::endl;
  cv::waitKey(0);
  
  // =========================================================================================================
  // 8. 清理和总结
  // Cleanup and summary
  // =========================================================================================================
  
  std::cout << "\n\n8. 清理和总结:" << std::endl;
  std::cout << "======================\n" << std::endl;
  
  // 释放智能指针
  smart_mat.release();
  if (smart_mat.empty()) {
    std::cout << "智能指针已释放" << std::endl;
  }
  
  // 清空处理器列表
  processors.clear();
  std::cout << "处理器列表已清空" << std::endl;
  
  std::cout << "\n========== 程序结束 ==========" << std::endl;
  std::cout << "总结:" << std::endl;
  std::cout << "1. Mat: OpenCV核心数据结构，用于存储图像和矩阵" << std::endl;
  std::cout << "2. Point: 表示2D/3D点，支持坐标运算" << std::endl;
  std::cout << "3. Size: 表示尺寸，包含宽度和高度" << std::endl;
  std::cout << "4. Rect: 表示矩形区域，支持交集和并集运算" << std::endl;
  std::cout << "5. Scalar: 表示颜色或标量，BGR顺序" << std::endl;
  std::cout << "6. Vec: 多通道数据，用于像素和向量运算" << std::endl;
  std::cout << "7. Range: 表示连续整数序列，用于选择子区域" << std::endl;
  std::cout << "8. Ptr: 智能指针，自动管理内存" << std::endl;
  
  return 0;
}






