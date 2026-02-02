#include<opencv2/opencv.hpp>
#include<iostream>
#include<string>

class VideoPlayer {
private:
  cv::Mat frame;
  std::string filename;
public:
  void write(std::string loadname_);
  void address(std::string name);
  bool adjust();
  cv::Mat& GetFrame();
  void toGray();
  void resize(float num);
};

int main() {
  VideoPlayer player1;
  float num;
  std::cin >> num;
  num = num > 0 && num <= 2.0 ? num : 1.0;
  player1.address("/home/yinkangan/Desktop/pic.jpg");
  if (player1.adjust()) {
    player1.toGray();
    player1.resize(num);
    cv::imshow("gray",player1.GetFrame()); 
  }

  cv::waitKey(0);

  std::string _name;
  std::cin >> _name;
  player1.write(_name);

  return 0;
}

//----------------------------------------------------------------------
cv::Mat& VideoPlayer::GetFrame() {
  return frame;
}

void VideoPlayer::address(std::string name) {
  frame = cv::imread(name);
}

bool VideoPlayer::adjust() {
  if(frame.empty()) {
    std::cerr << "Error: Image is empty." << std::endl;
    return false;
  } else {
    return true;
  }
} 

void VideoPlayer::toGray() {
  cv::cvtColor(frame,frame,cv::COLOR_BGR2GRAY);
}

void VideoPlayer::resize(float num) {
  cv::resize(frame,frame,cv::Size(frame.cols*num,frame.rows*num));
}

void VideoPlayer::write(std::string loadname_) {
  filename = loadname_;
  cv::imwrite(filename,frame);
}