#include <pose_graph/parameters.h>

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_match_points;
int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
int ROW;
int COL;
std::string VINS_RESULT_PATH;
int DEBUG_IMAGE;
int FAST_RELOCALIZATION;
