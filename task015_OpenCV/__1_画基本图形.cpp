/**************************
 * author:YinKang'an
 * date:2026.01.25
 **************************/

#include<opencv2/opencv.hpp>
#include<iostream>

int main()
{
  // 创建画布
  cv::Mat canvas = cv::Mat::zeros(600, 800, CV_8UC3);
  // 定义颜色
  cv::Scalar red(0, 0, 255);      // BGR: 红色
  cv::Scalar green(0, 255, 0);    // BGR: 绿色
  cv::Scalar blue(255, 0, 0);     // BGR: 蓝色
  cv::Scalar white(255, 255, 255);// BGR: 白色
  cv::Scalar yellow(0, 255, 255); // BGR: 黄色


  //----------------------------------------------------------------------------------
  // CV_EXPORTS_W void line(InputOutputArray img, Point pt1, Point pt2, const Scalar& color,
  //                    int thickness = 1, int lineType = LINE_8, int shift = 0);


  // 1.1 绘制水平直线
  cv::line(canvas, cv::Point(50, 50), cv::Point(200, 50), red, 2);
  // 1.2 绘制另一条直线
  cv::line(canvas, cv::Point(50, 80), cv::Point(200, 80), green, 2, cv::LINE_8);
  // 1.3 绘制对角线
  cv::line(canvas, cv::Point(50, 100), cv::Point(200, 200), blue, 3, cv::LINE_AA);

  //----------------------------------------------------------------------------------
  //CV_EXPORTS_W void rectangle(InputOutputArray img, Point pt1, Point pt2,
  //                        const Scalar& color, int thickness = 1,
  //                        int lineType = LINE_8, int shift = 0);
  //------------                                                         -------------
  //CV_EXPORTS_W void rectangle(InputOutputArray img, Rect rec,
  //                        const Scalar& color, int thickness = 1,
  //                        int lineType = LINE_8, int shift = 0);

  // 2.1 绘制空心矩形
  cv::rectangle(canvas, cv::Point(250, 50), cv::Point(400, 150), green, 2);
  // 2.2 绘制实心矩形（填充）
  cv::rectangle(canvas, cv::Point(450, 50), cv::Point(600, 150), cv::Scalar(100, 100, 255), -1);  // -1表示填充
  // 2.3 绘制正方形
  int side = 100;  // 边长
  cv::rectangle(canvas, cv::Point(650, 50), cv::Point(650 + side, 50 + side), yellow, 2); 

  //----------------------------------------------------------------------------------
  // CV_EXPORTS_W void circle(InputOutputArray img, Point center, int radius,
  //                       const Scalar& color, int thickness = 1,
  //                       int lineType = LINE_8, int shift = 0);

  // 3.1 绘制空心圆
  cv::circle(canvas, cv::Point(100, 300), 50, blue, 3);
  // 3.2 绘制实心圆
  cv::circle(canvas, cv::Point(250, 300), 50, cv::Scalar(255, 100, 100), -1);  // -1表示填充
  // 3.3 绘制抗锯齿的圆
  cv::circle(canvas, cv::Point(400, 300), 50, red, 2, cv::LINE_AA);

  //----------------------------------------------------------------------------------
  // CV_EXPORTS_W void ellipse(InputOutputArray img, Point center, Size axes, double angle,
  //                       double startAngle, double endAngle, const Scalar& color,
  //                       int thickness = 1, int lineType = LINE_8, int shift = 0);

  // 4.1 绘制完整椭圆
  cv::ellipse(canvas, cv::Point(100, 450), cv::Size(80, 50), 0, 0, 360, green, 2);
  // 4.2 绘制旋转45度的椭圆
  cv::ellipse(canvas, cv::Point(250, 450), cv::Size(80, 50), 45, 0, 360, blue, 2);
  // 4.3 绘制椭圆弧
  cv::ellipse(canvas, cv::Point(400, 450), cv::Size(80, 50), 0, 30, 270, red, 2);

  //----------------------------------------------------------------------------------
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

  //----------------------------------------------------------------------------------
  // 6.1 定义三角形顶点
  std::vector<cv::Point> triangle;
  triangle.push_back(cv::Point(650, 350));
  triangle.push_back(cv::Point(750, 350));
  triangle.push_back(cv::Point(700, 450));
  
  // 6.2 逐个连接顶点（空心三角形）
  for (int i = 0; i < 3; i++) {
    cv::line(canvas, triangle[i], triangle[(i + 1) % 3], white, 2);
  }
  
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

  //----------------------------------------------------------------------------------
  // CV_EXPORTS_W void arrowedLine(InputOutputArray img, Point pt1, Point pt2, const Scalar& color,
  //                           int thickness = 1, int line_type = 8, int shift = 0, double tipLength = 0.1);

  // 7.1 绘制水平箭头
  cv::arrowedLine(canvas, cv::Point(500, 550), cv::Point(600, 550), green, 3, 8, 0, 0.1);
  // 7.2 绘制斜向箭头
  cv::arrowedLine(canvas, cv::Point(500, 580), cv::Point(600, 630), blue, 3, cv::LINE_AA, 0, 0.1);

  //----------------------------------------------------------------------------------
  // CV_EXPORTS_W void putText(InputOutputArray img, const String& text, Point org,
  //                       int fontFace, double fontScale, Scalar color,
  //                       int thickness = 1, int lineType = LINE_8, bool bottomLeftOrigin = false);

  // 8.1 添加各种标签
  cv::putText(canvas, "Lines", cv::Point(50, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Rectangles", cv::Point(250, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Circles", cv::Point(50, 280), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Ellipses", cv::Point(50, 430), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Rotated Rect", cv::Point(500, 430), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Polygons", cv::Point(200, 520), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);
  cv::putText(canvas, "Arrows", cv::Point(500, 520), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 1);

  //----------------------------------------------------------------------------------
  // CV_EXPORTS_W void imshow(const String& winname, InputArray mat);

  // 9.1 显示图像
  cv::imshow("OpenCV Basic Shapes Drawing", canvas);
  // 9.2 等待按键
  cv::waitKey(0);
  // 9.3 保存到文件
  // cv::imwrite("basic_shapes.jpg", canvas);

  return 0;
}