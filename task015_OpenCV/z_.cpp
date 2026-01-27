#include<opencv2/opencv.hpp>

int pos = 0;
cv::Mat img;

void f(int pos, void*) {
  cv::Mat i = img.clone();
  i *= pos/1000000000.0;
  imshow("img",i);
}

int main() {

  img = cv::imread("/home/yinkangan/Desktop/pic.jpg");
  cv::namedWindow("img");
  cv::createTrackbar("pos","img",&pos,1000000000,f);

  f(pos,nullptr);

  cv::waitKey(0);
  return 0;
}