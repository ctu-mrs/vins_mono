#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int main() {
  Mat image = Mat::zeros(800, 848, CV_8UC1);
  circle(image, cv::Point2f(424, 400), 424, Scalar(255), -1, 8, 0);
  imwrite("mask.jpg", image);
  imshow("circle", image);
  waitKey(0);
  return 0;
}
