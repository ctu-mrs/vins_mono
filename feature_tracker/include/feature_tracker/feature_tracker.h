#ifndef FEATURE_TRACKER_H 
#define FEATURE_TRACKER_H 

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include <vins_mono_camera_model/camodocal/camera_models/CameraFactory.h>
#include <vins_mono_camera_model/camodocal/camera_models/CataCamera.h>
#include <vins_mono_camera_model/camodocal/camera_models/PinholeCamera.h>

#include <feature_tracker/parameters.h>
#include <feature_tracker/tic_toc.h>

using namespace std;
using namespace camodocal;
using namespace Eigen;

using namespace vins_mono::feature_tracker;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);

void reduceVector(vector<int> &v, vector<uchar> status);

bool compareFASTResponse(cv::KeyPoint first, cv::KeyPoint second);

/*//{ class FeatureTracker */
class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    size_t n_pts_tracked = 0;
    size_t n_pts_after_ransac = 0;
    size_t n_pts_added = 0;
    size_t n_pts_final = 0;

    double t_clahe = 0.0;
    double t_optical_flow = 0.0;
    double t_mask = 0.0;
    double t_ransac = 0.0;
    double t_detect_features = 0.0;
    double t_add_features = 0.0;
    double t_undistort = 0.0;

    static int n_id;
};
/*//}*/

#endif
