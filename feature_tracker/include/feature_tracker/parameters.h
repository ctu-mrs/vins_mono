#pragma once

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#if USE_MRS_LIB
#include <mrs_lib/param_loader.h>
#endif

namespace vins_mono {
  namespace feature_tracker {

const std::string NODE_NAME("FeatureTracker");

extern int ROW;
extern int COL;
extern double FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);

}
}
