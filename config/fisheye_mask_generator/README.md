## How to set
In file `generate.cpp` change height and width to the required.
The default are 640x480
  Mat image = Mat::zeros(480, 640, CV_8UC1);
  circle(image, cv::Point2f(320, 240), 320, Scalar(255), -1, 8, 0);

For T625 (848x800)
  Mat image = Mat::zeros(800, 848, CV_8UC1);
  circle(image, cv::Point2f(424, 400), 424, Scalar(255), -1, 8, 0);
## How to compile
reqires Open CV installed

g++ generate.cpp -o executable `pkg-config --cflags --libs opencv`
