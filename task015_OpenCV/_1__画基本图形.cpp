/********************************************************************
 * @author @b    YinKang'an
 * @date         2026.01.25
 * @brief        绘制基本图形
 * 
 * @date         2026.01.29
 * ------------------------------------------------------------------
 * @attention @b [声明说明]
 * \b            本代码中保留的中文注释+函数声明，仅用于鼠标悬停查看中文说明，直接编译可能会因类型别名导致冲突报错；
 * \b            若需实际编译运行，可参考下面方法：
 * \b            1.删掉在本文件下方声明，仅保留中文注释，调用时用cv::imread(会显示官方英文注释)
 * 
 * \b            某些OpenCV函数可直接仿照本注释格式添加中文说明，因参数类型无歧义，不会触发编译冲突
 * ------------------------------------------------------------------
 * @note         所有函数都重写了中文注释,鼠标光标移动到参数名上即可查看中文注释
 *               移动到函数名上可查看原始官方英文注释
 *               移动到参数类型上可查看原始官方英文参数类型
 ********************************************************************/

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;

int main() {
  // 创建画布
  cv::Mat canvas = cv::Mat::zeros(600, 800, CV_8UC3);
  // 定义颜色
  cv::Scalar red(0, 0, 255);      // BGR: 红色
  cv::Scalar green(0, 255, 0);    // BGR: 绿色
  cv::Scalar blue(255, 0, 0);     // BGR: 蓝色
  cv::Scalar white(255, 255, 255);// BGR: 白色
  cv::Scalar yellow(0, 255, 255); // BGR: 黄色



//--------------------------------------------------------------------------
/**
 * @brief 绘制一条线段连接两个点
 * 
 * 该函数在图像上绘制一条从点 pt1 到点 pt2 的线段，线段颜色由 color 指定，
 * 支持设置线条宽度、线型和坐标精度偏移量。线段会被剪裁至图像边界内。
 * 
 * @param img [in/out] 输入输出图像，绘制的线段会直接修改该图像
 * @param pt1 [in] 线段的第一个端点坐标 (x1, y1)
 * @param pt2 [in] 线段的第二个端点坐标 (x2, y2)
 * @param color [in] 线段的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 线段的宽度，默认值为1；若设为 FILLED(-1) 则填充线段区域（仅对特殊线型有效）
 * @param lineType [in] 线段的类型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑边缘）
 * @param shift [in] 坐标点的小数位数偏移量，默认值为0（坐标为整数）
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像；
 *       3. 当 thickness 过大时，线段端点可能出现锯齿，建议配合 LINE_AA 使用。
 * @sa cv::rectangle, cv::circle, cv::ellipse
 */
CV_EXPORTS_W void line(InputOutputArray img, Point pt1, Point pt2, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
  // 1.1 绘制水平直线
  cv::line(canvas, cv::Point(50, 50), cv::Point(200, 50), red, 2);
  // 1.2 绘制另一条直线
  cv::line(canvas, cv::Point(50, 80), cv::Point(200, 80), green, 2, cv::LINE_8);
  // 1.3 绘制对角线
  cv::line(canvas, cv::Point(50, 100), cv::Point(200, 200), blue, 3, cv::LINE_AA);

//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
/**
 * @brief 绘制矩形轮廓或填充矩形
 * 
 * 该函数在图像上绘制矩形，矩形的两个对角顶点由 pt1 和 pt2 指定，
 * 支持设置轮廓宽度、线型和坐标精度偏移量。矩形会被剪裁至图像边界内。
 * 
 * @param img [in/out] 输入输出图像，绘制的矩形会直接修改该图像
 * @param pt1 [in] 矩形的一个对角顶点坐标 (x1, y1)
 * @param pt2 [in] 矩形的另一个对角顶点坐标 (x2, y2)
 * @param color [in] 矩形的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 矩形轮廓的宽度，默认值为1；若设为 FILLED(-1) 则填充矩形区域
 * @param lineType [in] 轮廓线的类型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑边缘）
 * @param shift [in] 顶点坐标的小数位数偏移量，默认值为0（坐标为整数）
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像；
 *       3. 当 thickness 过大时，矩形轮廓端点可能出现锯齿，建议配合 LINE_AA 使用。
 * @sa cv::line, cv::circle, cv::ellipse
 */
CV_EXPORTS_W void rectangle(InputOutputArray img, Point pt1, Point pt2, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
/**
 * @brief 绘制矩形轮廓或填充矩形
 * 
 * 该函数在图像上绘制矩形，矩形由 Rect 结构体 rec 指定，
 * 支持设置轮廓宽度、线型和坐标精度偏移量。矩形会被剪裁至图像边界内。
 * 
 * @param img [in/out] 输入输出图像，绘制的矩形会直接修改该图像
 * @param rec [in] 描述矩形的 Rect 结构体，包含矩形的位置和尺寸信息
 * @param color [in] 矩形的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 矩形轮廓的宽度，默认值为1；若设为 FILLED(-1) 则填充矩形区域
 * @param lineType [in] 轮廓线的类型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑边缘）
 * @param shift [in] 矩形坐标的小数位数偏移量，默认值为0（坐标为整数）
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像；
 *       3. 当 thickness 过大时，矩形轮廓边缘可能出现锯齿，建议配合 LINE_AA 使用。
 * @sa cv::line, cv::circle, cv::ellipse, cv::Rect
 */
CV_EXPORTS_W void rectangle(InputOutputArray img, Rect rec, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
  // 2.1 绘制空心矩形
  cv::rectangle(canvas, cv::Point(250, 50), cv::Point(400, 150), green, 2);
  // 2.2 绘制实心矩形（填充）
  cv::rectangle(canvas, cv::Point(450, 50), cv::Point(600, 150), cv::Scalar(100, 100, 255), -1);  // -1表示填充
  // 2.3 绘制正方形
  int side = 100;  // 边长
  cv::rectangle(canvas, cv::Point(650, 50), cv::Point(650 + side, 50 + side), yellow, 2); 
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
/**
 * @brief 绘制圆形轮廓或填充圆形
 * 
 * 该函数在图像上绘制圆形，圆形的圆心由 center 指定、半径由 radius 指定，
 * 支持设置轮廓宽度、线型和坐标精度偏移量。圆形会被剪裁至图像边界内。
 * 
 * @param img [in/out] 输入输出图像，绘制的圆形会直接修改该图像
 * @param center [in] 圆形的圆心坐标 (x, y)
 * @param radius [in] 圆形的半径（像素数），需为非负值
 * @param color [in] 圆形的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 圆形轮廓的宽度，默认值为1；若设为 FILLED(-1) 则填充圆形区域
 * @param lineType [in] 轮廓线的类型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑边缘）
 * @param shift [in] 圆心坐标和半径的小数位数偏移量，默认值为0（坐标/半径为整数）
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像；
 *       3. 当 thickness 过大时，圆形轮廓边缘可能出现锯齿，建议配合 LINE_AA 使用。
 * @sa cv::line, cv::rectangle, cv::ellipse
 */
CV_EXPORTS_W void circle(InputOutputArray img, Point center, int radius, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
  // 3.1 绘制空心圆
  cv::circle(canvas, cv::Point(100, 300), 50, blue, 3);
  // 3.2 绘制实心圆
  cv::circle(canvas, cv::Point(250, 300), 50, cv::Scalar(255, 100, 100), -1);  // -1表示填充
  // 3.3 绘制抗锯齿的圆
  cv::circle(canvas, cv::Point(400, 300), 50, red, 2, cv::LINE_AA);
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
/**
 * @brief 绘制椭圆轮廓或填充椭圆
 * 
 * 该函数在图像上绘制椭圆，椭圆的圆心由 center 指定，轴尺寸由 axes 指定，
 * 支持设置旋转角度、绘制角度范围、轮廓宽度、线型和坐标精度偏移量。椭圆会被剪裁至图像边界内。
 * 
 * @param img [in/out] 输入输出图像，绘制的椭圆会直接修改该图像
 * @param center [in] 椭圆的圆心坐标 (x, y)
 * @param axes [in] 椭圆的长轴和短轴尺寸，Size(width, height) 分别对应横轴和纵轴长度
 * @param angle [in] 椭圆整体的旋转角度（度），顺时针方向为正
 * @param startAngle [in] 椭圆绘制的起始角度（度），从长轴方向开始计算
 * @param endAngle [in] 椭圆绘制的结束角度（度），若为360则绘制完整椭圆
 * @param color [in] 椭圆的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 椭圆轮廓的宽度，默认值为1；若设为 FILLED(-1) 则填充椭圆区域
 * @param lineType [in] 轮廓线的类型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑边缘）
 * @param shift [in] 圆心坐标和轴尺寸的小数位数偏移量，默认值为0（坐标/尺寸为整数）
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像；
 *       3. 当 thickness 过大时，椭圆轮廓边缘可能出现锯齿，建议配合 LINE_AA 使用。
 * @sa cv::line, cv::rectangle, cv::circle
 */
CV_EXPORTS_W void ellipse(InputOutputArray img, Point center, Size axes, double angle, double startAngle, double endAngle, const Scalar& color, int thickness = 1, int lineType = LINE_8, int shift = 0);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
  // 4.1 绘制完整椭圆
  cv::ellipse(canvas, cv::Point(100, 450), cv::Size(80, 50), 0, 0, 360, green, 2);
  // 4.2 绘制旋转45度的椭圆
  cv::ellipse(canvas, cv::Point(250, 450), cv::Size(80, 50), 45, 0, 360, blue, 2);
  // 4.3 绘制椭圆弧
  cv::ellipse(canvas, cv::Point(400, 450), cv::Size(80, 50), 0, 30, 270, red, 2);
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
  // 5.1 创建RotatedRect对象
  cv::Point center(550, 450);
  cv::Size rectSize(120, 80);
  double angle = 30;  // 旋转30度
  cv::RotatedRect rRect(center, rectSize, angle);
  cv::Point2f vertices[4];
  rRect.points(vertices);
  
  // 5.2 连接四个顶点形成矩形
  for (int i = 0; i < 4; i++) {
    cv::line(canvas, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 255), 2);
  }
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
  // 6.1 定义三角形顶点
  std::vector<cv::Point> triangle;
  triangle.push_back(cv::Point(650, 350));
  triangle.push_back(cv::Point(750, 350));
  triangle.push_back(cv::Point(700, 450));
  
  // 6.2 逐个连接顶点（空心三角形）
  for (int i = 0; i < 3; i++) {
    cv::line(canvas, triangle[i], triangle[(i + 1) % 3], white, 2);
  }
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
  // 6.3 绘制五边形
  std::vector<cv::Point> pentagon;
  for (int i = 0; i < 5; i++) {
    double theta = i * 72 * CV_PI / 180.0;
    int x = 200 + 50 * cos(theta);
    int y = 550 + 50 * sin(theta);
    pentagon.push_back(cv::Point(x, y));
  }
  
  std::vector<std::vector<cv::Point>> pentagonContours;
  pentagonContours.push_back(pentagon);
  cv::polylines(canvas, pentagonContours, true, cv::Scalar(0, 255, 255), 2);
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
  // 6.4 绘制实心六边形
  std::vector<cv::Point> hexagon;
  for (int i = 0; i < 6; i++) {
    double theta = i * 60 * CV_PI / 180.0;
    int x = 400 + 50 * cos(theta);
    int y = 550 + 50 * sin(theta);
    hexagon.push_back(cv::Point(x, y));
  }
  
  std::vector<std::vector<cv::Point>> hexagonContours;
  hexagonContours.push_back(hexagon);
  cv::fillPoly(canvas, hexagonContours, cv::Scalar(100, 100, 255));
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
/**
 * @brief 绘制带箭头的线段
 * 
 * 该函数在图像上绘制一条从点 pt1 到点 pt2 的带箭头线段，线段颜色由 color 指定，
 * 支持设置线条宽度、线型、坐标精度偏移量和箭头长度比例。线段会被剪裁至图像边界内。
 * 
 * @param img [in/out] 输入输出图像，绘制的带箭头线段会直接修改该图像
 * @param pt1 [in] 线段的起点坐标 (x1, y1)
 * @param pt2 [in] 线段的终点（箭头指向）坐标 (x2, y2)
 * @param color [in] 线段和箭头的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 线段和箭头的宽度，默认值为1；若设为 FILLED(-1) 则填充箭头区域（仅对特殊线型有效）
 * @param line_type [in] 线段的类型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑边缘）
 * @param shift [in] 坐标点的小数位数偏移量，默认值为0（坐标为整数）
 * @param tipLength [in] 箭头长度占整条线段长度的比例，默认值为0.1（范围0~1）
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像；
 *       3. 当 thickness 过大时，箭头端点可能出现锯齿，建议配合 LINE_AA 使用。
 * @sa cv::line, cv::rectangle, cv::circle
 */
CV_EXPORTS_W void arrowedLine(InputOutputArray img, Point pt1, Point pt2, const Scalar& color, int thickness = 1, int line_type = 8, int shift = 0, double tipLength = 0.1);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
  // 7.1 绘制水平箭头
  cv::arrowedLine(canvas, cv::Point(500, 550), cv::Point(600, 550), green, 3, 8, 0, 0.1);
  // 7.2 绘制斜向箭头
  cv::arrowedLine(canvas, cv::Point(500, 580), cv::Point(600, 630), blue, 3, cv::LINE_AA, 0, 0.1);
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
/**
 * @brief 在图像上绘制文本字符串
 * 
 * 该函数在图像指定位置绘制文本字符串，支持设置字体类型、缩放比例、颜色、线条宽度等参数，
 * 绘制的文本会被剪裁至图像边界内。
 * 
 * @attention 文本字符只支持英文，不支持中文字符(输入中文会出现"???????")
 * 
 * @param img [in/out] 输入输出图像，绘制的文本会直接修改该图像
 * @param text [in] 要绘制的文本字符串
 * @param org [in] 文本字符串的绘制起始坐标，默认指文本左上角位置 (x, y)
 * @param fontFace [in] 字体类型，可选值包括 FONT_HERSHEY_SIMPLEX、FONT_HERSHEY_PLAIN 等
 * @param fontScale [in] 字体缩放比例，大于1放大字体，小于1缩小字体
 * @param color [in] 文本的颜色，BGR 格式（如 Scalar(255,0,0) 表示蓝色）
 * @param thickness [in] 文本笔画的宽度，默认值为1；数值越大文本越粗
 * @param lineType [in] 文本笔画的线型，可选值：
 *        - LINE_4: 4邻域连接线型
 *        - LINE_8: 8邻域连接线型（默认）
 *        - LINE_AA: 抗锯齿线型（平滑文本边缘）
 * @param bottomLeftOrigin [in] 文本坐标原点标识，默认值false（原点为左上角）；设为true则原点为左下角
 * 
 * @note 1. 若图像为多通道，则 color 的通道数需与图像通道数匹配；
 *       2. 抗锯齿线型 (LINE_AA) 仅支持 8位单通道或 BGR 彩色图像，可优化文本边缘效果；
 *       3. 当 fontScale 过大时，文本易出现锯齿，建议配合 LINE_AA 使用
 * @sa cv::getTextSize, cv::line, cv::rectangle
 */
CV_EXPORTS_W void putText(InputOutputArray img, const String& text, Point org, int fontFace, double fontScale, Scalar color, int thickness = 1, int lineType = LINE_8, bool bottomLeftOrigin = false);
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
  // 8.1 添加各种标签
  cv::putText(canvas, "Lines", cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Rectangles", cv::Point(250, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Circles", cv::Point(50, 280), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Ellipses", cv::Point(50, 430), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Rotated Rect", cv::Point(500, 430), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Polygons", cv::Point(200, 520), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Arrows", cv::Point(500, 520), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
//--------------------------------------------------------------------------



//--------------------------------------------------------------------------
  // 9.1 显示图像
  cv::imshow("OpenCV Basic Shapes Drawing", canvas);
  // 9.2 等待按键
  cv::waitKey(0);

  return 0;
}