#include <feature_tracker/parameters.h>

namespace vins_mono {
  namespace feature_tracker {

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
std::string FISHEYE_MASK_NAME;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int FEATURE_DETECTOR;
int FAST_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
double FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

#if USE_MRS_LIB

/*//{ mrs_lib readParameters() */
void readParameters(ros::NodeHandle &n)
{
    ROS_INFO("[%s]: loading parameters using ParamLoader", NODE_NAME.c_str());
    mrs_lib::ParamLoader pl(n, NODE_NAME);

    std::string config_file;
    pl.loadParam("config_file", config_file);
    pl.addYamlFile(config_file);

    std::string CONFIG_PATH;
    pl.loadParam("config_path", CONFIG_PATH);

    pl.loadParam("feature_detector", FEATURE_DETECTOR);
    pl.loadParam("fast_threshold", FAST_THRESHOLD);
    pl.loadParam("max_cnt", MAX_CNT);
    pl.loadParam("min_dist", MIN_DIST);
    pl.loadParam("image_height", ROW);
    pl.loadParam("image_width", COL);
    pl.loadParam("freq", FREQ);
    pl.loadParam("F_threshold", F_THRESHOLD);
    pl.loadParam("show_track", SHOW_TRACK);
    pl.loadParam("equalize", EQUALIZE);
    pl.loadParam("fisheye", FISHEYE);
    pl.loadParam("fisheye_mask_name", FISHEYE_MASK_NAME);
    if (FISHEYE == 1) 
    {
      FISHEYE_MASK = CONFIG_PATH + "/" + FISHEYE_MASK_NAME;
    }

    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

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

    if (FREQ == 0)
    {
      FREQ = 100;
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
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string CONFIG_PATH = readParam<std::string>(n, "config_path");

    FEATURE_DETECTOR = FeatureDetector_t::GFTT;
    FEATURE_DETECTOR = fsSettings["feature_detector"];
    FAST_THRESHOLD = 30;
    FAST_THRESHOLD = fsSettings["fast_threshold"];
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    fsSettings["fisheye_mask_name"] >> FISHEYE_MASK_NAME;
    if (FISHEYE == 1)
        FISHEYE_MASK = CONFIG_PATH + "/"+ FISHEYE_MASK_NAME;
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    PUB_THIS_FRAME = false;

    cv::FileNode projection_parameters = fsSettings["projection_parameters"];
    double mu = static_cast<double>(projection_parameters["mu"]);
    double mv = static_cast<double>(projection_parameters["mv"]);
    double m = (mu + mv) / 2.0;

    double fx = static_cast<double>(projection_parameters["fx"]);
    double fy = static_cast<double>(projection_parameters["fy"]);
    double f = (fx + fy) / 2.0;

    FOCAL_LENGTH = m > f ? m : f;
    ROS_INFO("[%s]: FOCAL_LENGTH: %.2f", ros::this_node::getName().c_str(), FOCAL_LENGTH);

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
}
/*//}*/

#endif
}
}
