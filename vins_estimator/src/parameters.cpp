#include <vins_estimator/parameters.h>

namespace vins_mono {
  namespace vins_estimator {

std::string UAV_NAME;
std::string VINS_WORLD_FRAME_ID;

double INIT_DEPTH;
double MIN_PARALLAX;
bool ROTATION_COMPENSATED_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

double FOCAL_LENGTH;

double INIT_MIN_PARALLAX;
int INIT_MIN_FEATURES;
double INIT_MIN_IMU_VARIANCE;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int SOLVER_THREADS;
int NUM_ITERATIONS;
int LOSS_FUNCTION;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
double ROW, COL;
double TD, TR;

bool WRITE_EXTRINSICS_TO_FILE;
bool WRITE_RESULTS_TO_FILE;

#if USE_MRS_LIB

/*//{ mrs_lib readParameters() */
void readParameters(ros::NodeHandle &n)
{

    ROS_INFO("[%s]: loading parameters using ParamLoader", NODE_NAME.c_str());
    mrs_lib::ParamLoader pl(n, NODE_NAME);

    pl.loadParam("uav_name", UAV_NAME);
    pl.loadParam("vins_world_frame_id", VINS_WORLD_FRAME_ID);

    std::string config_file;
    pl.loadParam("config_file", config_file);

    pl.addYamlFile(config_file);

    std::string model_type;
    pl.loadParam("model_type", model_type);
    if (model_type == "KANNALA_BRANDT" || model_type == "SCARAMUZZA") 
    {
      double mu, mv;
      pl.loadParam("projection_parameters/mu", mu);
      pl.loadParam("projection_parameters/mv", mv);
      FOCAL_LENGTH = (mu + mv) / 2.0;
    }
    else 
    {
      double fx, fy;
      pl.loadParam("projection_parameters/fx", fx);
      pl.loadParam("projection_parameters/fy", fy);
      FOCAL_LENGTH = (fx + fy) / 2.0;
    }

    ROS_INFO("[%s]: FOCAL_LENGTH: %.2f", NODE_NAME.c_str(), FOCAL_LENGTH);

    pl.loadParam("max_solver_time", SOLVER_TIME);
    pl.loadParam("solver_threads", SOLVER_THREADS);
    pl.loadParam("max_num_iterations", NUM_ITERATIONS);
    pl.loadParam("rotation_compensated_parallax", ROTATION_COMPENSATED_PARALLAX);
    pl.loadParam("keyframe_parallax", MIN_PARALLAX);
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    pl.loadParam("loss_function", LOSS_FUNCTION);
    if (LOSS_FUNCTION == LossFunction_t::HUBER)
    {
        ROS_INFO("[%s]: Using Huber loss function.", NODE_NAME.c_str());
    }
    else if (LOSS_FUNCTION == LossFunction_t::CAUCHY)
    {
        ROS_INFO("[%s]: Using Cauchy loss function.", NODE_NAME.c_str());
    }
    else 
    {
        ROS_ERROR("[%s]: Unkown loss function: %d. Shutting down.", NODE_NAME.c_str(), LOSS_FUNCTION);
        ros::shutdown();
    }

    pl.loadParam("init_min_parallax", INIT_MIN_PARALLAX);
    pl.loadParam("init_min_features", INIT_MIN_FEATURES);
    pl.loadParam("init_min_imu_variance", INIT_MIN_IMU_VARIANCE);


    pl.loadParam("acc_n", ACC_N);
    pl.loadParam("acc_w", ACC_W);
    pl.loadParam("gyr_n", GYR_N);
    pl.loadParam("gyr_w", GYR_W);

    double gz;
    pl.loadParam("g_norm", gz);
    G.z() = gz;

    pl.loadParam("image_height", ROW);
    pl.loadParam("image_width", COL);

    pl.loadParam("estimate_extrinsic", ESTIMATE_EXTRINSIC);
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("[%s]: Have no prior about extrinsic param, calibrate extrinsic param", NODE_NAME.c_str());
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_INFO("[%s]: Optimize extrinsic param around initial guess!", NODE_NAME.c_str());
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_INFO("[%s]: Fixed extrinsic param. No online optimization of extrinsic params.", NODE_NAME.c_str());

        /* cv::Mat cv_R, cv_T; */
        /* fsSettings["extrinsicRotation"] >> cv_R; */
        /* fsSettings["extrinsicTranslation"] >> cv_T; */
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        /* cv::cv2eigen(cv_R, eigen_R); */
        /* cv::cv2eigen(cv_T, eigen_T); */
        pl.loadMatrixStatic("extrinsicRotation/data", eigen_R);
        pl.loadMatrixStatic("extrinsicTranslation/data", eigen_T);
        // note petrlmat: why convert to quaternion and back?
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO("[%s]: Extrinsic_R :", NODE_NAME.c_str());
        ROS_INFO_STREAM(std::endl << RIC[0]);
        ROS_INFO("[%s]: Extrinsic_T :", NODE_NAME.c_str());
        ROS_INFO_STREAM(TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    pl.loadParam("td", TD);
    pl.loadParam("estimate_td", ESTIMATE_TD);
    if (ESTIMATE_TD)
        ROS_INFO("[%s]: Unsynchronized sensors, online estimate time offset, initial td: %.4f s", NODE_NAME.c_str(), TD);
    else
        ROS_INFO("[%s]: Synchronized sensors, fixed time offset: %.4f s", NODE_NAME.c_str(), TD);

    pl.loadParam("rolling_shutter", ROLLING_SHUTTER);
    if (ROLLING_SHUTTER)
    {
        pl.loadParam("rolling_shutter_tr", TR);
        ROS_INFO("[%s]: Rolling shutter camera, read out time per frame: %.4f s", NODE_NAME.c_str(), TR);
    }
    else
    {
        ROS_INFO("[%s]: Global shutter camera", NODE_NAME.c_str());
        TR = 0;
    }

    // Output path is loaded from parameters to be able to change the path from launch file and avoid absolute path in configs
    pl.loadParam("write_results_to_file", WRITE_RESULTS_TO_FILE);
    pl.loadParam("write_extrinsics_to_file", WRITE_EXTRINSICS_TO_FILE);
    std::string OUTPUT_PATH;
    if (WRITE_RESULTS_TO_FILE || WRITE_EXTRINSICS_TO_FILE)
    {
        pl.loadParam("output_path", OUTPUT_PATH);
    }
    if (WRITE_RESULTS_TO_FILE)
    {
        VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
    }

    if (WRITE_EXTRINSICS_TO_FILE && (ESTIMATE_EXTRINSIC == 1 || ESTIMATE_EXTRINSIC == 2))
    {
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        std::ofstream fout(EX_CALIB_RESULT_PATH, std::ios::out);
        fout.close();
    }

    if (!pl.loadedSuccessfully()) 
    {
      ROS_ERROR("[VinsEstimator]: Could not load all non-optional parameters. Shutting down.");
      ros::shutdown();
    }
}
/*//}*/

#else

/*//{ readParam() */
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}
/*//}*/

/*//{ OG readParameters() */
void readParameters(ros::NodeHandle &n)
{

    n.param<std::string>("uav_name", UAV_NAME); // loaded from launch file, not config file
    n.param<std::string>("vins_world_frame_id", VINS_WORLD_FRAME_ID); // loaded from launch file, not config file

    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cv::FileNode projection_parameters = fsSettings["projection_parameters"];
    double mu = static_cast<double>(projection_parameters["mu"]);
    double mv = static_cast<double>(projection_parameters["mv"]);
    double m = (mu + mv) / 2.0;

    double fx = static_cast<double>(projection_parameters["fx"]);
    double fy = static_cast<double>(projection_parameters["fy"]);
    double f = (fx + fy) / 2.0;

    FOCAL_LENGTH = m > f ? m : f;
    ROS_INFO("[%s]: FOCAL_LENGTH: %.2f", NODE_NAME.c_str(), FOCAL_LENGTH);

    SOLVER_TIME = fsSettings["max_solver_time"];
    SOLVER_THREADS = fsSettings["solver_threads"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    ROTATION_COMPENSATED_PARALLAX = fsSettings["rotation_compensated_parallax"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    LOSS_FUNCTION = fsSettings["loss_function"];
    if (LOSS_FUNCTION == LossFunction_t::HUBER)
    {
        ROS_INFO("[%s]: Using Huber loss function.", NODE_NAME.c_str());
    }
    else if (LOSS_FUNCTION == LossFunction_t::CAUCHY)
    {
        ROS_INFO("[%s]: Using Cauchy loss function.", NODE_NAME.c_str());
    }
    else 
    {
        ROS_ERROR("[%s]: Unkown loss function: %d. Shutting down.", NODE_NAME.c_str(), LOSS_FUNCTION);
        ros::shutdown();
    }

    INIT_MIN_PARALLAX = fsSettings["init_min_parallax"];
    INIT_MIN_FEATURES = fsSettings["init_min_features"];
    INIT_MIN_IMU_VARIANCE = fsSettings["init_min_imu_variance"];


    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" Fixed extrinsic param. No online optimization of extrinsic params. ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    // Output path is loaded from parameters to be able to change the path from launch file and avoid absolute path in configs
    WRITE_RESULTS_TO_FILE = fsSettings["write_results_to_file"];
    WRITE_EXTRINSICS_TO_FILE = fsSettings["write_extrinsics_to_file"];
    std::string OUTPUT_PATH;
    if (WRITE_RESULTS_TO_FILE || WRITE_EXTRINSICS_TO_FILE) 
    {
        OUTPUT_PATH = readParam<std::string>(n, "output_path");
        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());
    }

    if (WRITE_RESULTS_TO_FILE) 
    {
        VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
        std::cout << "result path " << VINS_RESULT_PATH << std::endl;
        // create file
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
    }

    if (WRITE_EXTRINSICS_TO_FILE && (ESTIMATE_EXTRINSIC == 1 || ESTIMATE_EXTRINSIC == 2))
    {
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        // create file
        std::ofstream fout(EX_CALIB_RESULT_PATH, std::ios::out);
        fout.close();
    }
    
    fsSettings.release();
}
/*//}*/

#endif


}
}
