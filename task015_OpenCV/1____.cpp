#include<opencv2/opencv.hpp>

int pos = 5000;
cv::Mat img;

void f(int pos, void*) {
  cv::Mat frame = img.clone();
  frame *= pos/10000.0;
  imshow("img",frame);
}

int main() {

  img = cv::imread("/home/yinkangan/Desktop/pic.jpg");
  cv::namedWindow("img");
  cv::createTrackbar("pos","img",&pos,10000,f);

  f(pos,nullptr);

  cv::waitKey(0);
  return 0;
}

