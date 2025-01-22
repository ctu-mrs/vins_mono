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
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
double FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

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

/*//{ readParameters() */
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

}
}
