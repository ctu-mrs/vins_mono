#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

#include <vins_estimator/utility/utility.h>

#if USE_MRS_LIB
#include <mrs_lib/param_loader.h>
#endif

namespace vins_mono {
  namespace vins_estimator {

typedef enum
{
  CAUCHY,
  HUBER
} LossFunction_t;

const std::string NODE_NAME("VinsEstimator");

const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double FOCAL_LENGTH;

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double INIT_MIN_PARALLAX;
extern int INIT_MIN_FEATURES;
extern double INIT_MIN_IMU_VARIANCE;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int SOLVER_THREADS;
extern int NUM_ITERATIONS;
extern int LOSS_FUNCTION;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

extern bool WRITE_EXTRINSICS_TO_FILE;
extern bool WRITE_RESULTS_TO_FILE;

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

}
}
