#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>

#include <vector>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pose_graph/keyframe.h>
#include <pose_graph/utility/tic_toc.h>
#include <pose_graph/pose_graph.h>
#include <pose_graph/utility/CameraPoseVisualization.h>
#include <pose_graph/parameters.h>

#define SKIP_FIRST_CNT 10

using namespace std;

namespace vins_mono
{
  
  using namespace pose_graph;

/*//{ class PoseGraphNodelet */
class PoseGraphNodelet : public nodelet::Nodelet {

public:
  virtual void onInit();

private:

  bool is_initialized_ = false;

  queue<sensor_msgs::ImageConstPtr> image_buf;
  queue<sensor_msgs::PointCloudConstPtr> point_buf;
  queue<nav_msgs::Odometry::ConstPtr> pose_buf;
  queue<Eigen::Vector3d> odometry_buf;
  std::mutex m_buf;
  std::mutex m_process;
  int frame_index  = 0;
  int sequence = 1;
  PoseGraph posegraph;
  int skip_first_cnt = 0;
  int SKIP_CNT;
  int skip_cnt = 0;
  bool load_flag = 0;
  bool start_flag = 0;
  double SKIP_DIS = 0;
  string uav_name = "";

  int VISUALIZE_IMU_FORWARD;
  int LOOP_CLOSURE;
  double FOCAL_LENGTH;

  nav_msgs::Path no_loop_path;

  CameraPoseVisualization cameraposevisual;
  Eigen::Vector3d last_t;
  double last_image_time = -1;

  ros::Publisher pub_camera_pose_visual;
  ros::Publisher pub_key_odometrys;
  ros::Publisher pub_vio_path;

  void newSequence();

  ros::Subscriber sub_image;
  void callbackImage(const sensor_msgs::ImageConstPtr &image_msg);

  ros::Subscriber sub_point;
  void callbackPoint(const sensor_msgs::PointCloudConstPtr &point_msg);

  ros::Subscriber sub_pose;
  void callbackPose(const nav_msgs::Odometry::ConstPtr &pose_msg);

  ros::Subscriber sub_imu_forward;
  void callbackImuForward(const nav_msgs::Odometry::ConstPtr &forward_msg);

  ros::Subscriber sub_relo_relative_pose;
  void callbackReloRelativePose(const nav_msgs::Odometry::ConstPtr &pose_msg);

  ros::Subscriber sub_vio;
  void callbackVio(const nav_msgs::Odometry::ConstPtr &pose_msg);

  ros::Subscriber sub_extrinsic;
  void callbackExtrinsic(const nav_msgs::Odometry::ConstPtr &pose_msg);

  void process();
  void command();
};
/*//}*/

/*//{ onInit() */
void PoseGraphNodelet::onInit()
{

    const std::string node_name("PoseGraph");

    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    ROS_INFO("[%s]: Initializing", node_name.c_str());

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    cameraposevisual = CameraPoseVisualization(1, 0, 0, 1);
    last_t = Eigen::Vector3d(-100, -100, -100);

    nh.param<std::string>("uav_name", uav_name, "uav1");

    posegraph.registerPub(nh);

    // read param
    nh.getParam("visualization_shift_x", VISUALIZATION_SHIFT_X);
    nh.getParam("visualization_shift_y", VISUALIZATION_SHIFT_Y);
    nh.getParam("skip_cnt", SKIP_CNT);
    nh.getParam("skip_dis", SKIP_DIS);
    std::string config_file;
    nh.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        ROS_ERROR("[%s]: Wrong path to settings!", ros::this_node::getName().c_str());
        return;
    }

    double camera_visual_size = fsSettings["visualize_camera_size"];
    cameraposevisual.setScale(camera_visual_size);
    cameraposevisual.setLineWidth(camera_visual_size / 10.0);

    cv::FileNode projection_parameters = fsSettings["projection_parameters"];
    double mu = static_cast<double>(projection_parameters["mu"]);
    double mv = static_cast<double>(projection_parameters["mv"]);
    double m = (mu + mv) / 2.0;

    double fx = static_cast<double>(projection_parameters["fx"]);
    double fy = static_cast<double>(projection_parameters["fy"]);
    double f = (fx + fy) / 2.0;

    FOCAL_LENGTH = m > f ? m : f;
    ROS_INFO("[%s]: FOCAL_LENGTH: %.2f", ros::this_node::getName().c_str(), FOCAL_LENGTH);

    posegraph.setFocalLength(FOCAL_LENGTH);


    LOOP_CLOSURE = fsSettings["loop_closure"];
    int LOAD_PREVIOUS_POSE_GRAPH;
    if (LOOP_CLOSURE)
    {
        ROW = fsSettings["image_height"];
        COL = fsSettings["image_width"];

        // TODO petrlmat: parametrize path to vocabulary in config
        std::string pkg_path = ros::package::getPath("vins_mono_pose_graph");
        string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
        ROS_INFO("[%s]: vocabulary file: %s", ros::this_node::getName().c_str(), vocabulary_file.c_str());
        posegraph.loadVocabulary(vocabulary_file);

        BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
        ROS_INFO("[%s]: brief pattern file: %s", ros::this_node::getName().c_str(), BRIEF_PATTERN_FILE.c_str());
        m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());

        fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
        fsSettings["output_path"] >> VINS_RESULT_PATH;
        fsSettings["save_image"] >> DEBUG_IMAGE;

        // create folder if not exists
        FileSystemHelper::createDirectoryIfNotExists(POSE_GRAPH_SAVE_PATH.c_str());
        FileSystemHelper::createDirectoryIfNotExists(VINS_RESULT_PATH.c_str());

        VISUALIZE_IMU_FORWARD = fsSettings["visualize_imu_forward"];
        LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
        FAST_RELOCALIZATION = fsSettings["fast_relocalization"];
        VINS_RESULT_PATH = VINS_RESULT_PATH + "/vins_result_loop.csv";
        std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
        fout.close();
        fsSettings.release();

        if (LOAD_PREVIOUS_POSE_GRAPH)
        {
            printf("load pose graph\n");
            m_process.lock();
            posegraph.loadPoseGraph();
            m_process.unlock();
            printf("load pose graph finish\n");
            load_flag = 1;
        }
        else
        {
            printf("no previous pose graph\n");
            load_flag = 1;
        }
    }

    fsSettings.release();

    sub_imu_forward = nh.subscribe("vins_estimator/imu_propagate", 1, &PoseGraphNodelet::callbackImuForward, this, ros::TransportHints().tcpNoDelay());
    sub_vio = nh.subscribe("vins_estimator/odometry", 1, &PoseGraphNodelet::callbackVio, this, ros::TransportHints().tcpNoDelay());
    sub_image = nh.subscribe("image_in", 1, &PoseGraphNodelet::callbackImage, this, ros::TransportHints().tcpNoDelay());
    sub_pose = nh.subscribe("vins_estimator/keyframe_pose", 1, &PoseGraphNodelet::callbackPose, this, ros::TransportHints().tcpNoDelay());
    sub_extrinsic = nh.subscribe("vins_estimator/extrinsic", 1, &PoseGraphNodelet::callbackExtrinsic, this, ros::TransportHints().tcpNoDelay());
    sub_point = nh.subscribe("vins_estimator/keyframe_point", 1, &PoseGraphNodelet::callbackPoint, this, ros::TransportHints().tcpNoDelay());
    sub_relo_relative_pose = nh.subscribe("vins_estimator/relo_relative_pose", 1, &PoseGraphNodelet::callbackReloRelativePose, this, ros::TransportHints().tcpNoDelay());

    pub_match_img = nh.advertise<sensor_msgs::Image>("match_image", 1);
    pub_camera_pose_visual = nh.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1);
    pub_key_odometrys = nh.advertise<visualization_msgs::Marker>("key_odometrys", 1);
    pub_vio_path = nh.advertise<nav_msgs::Path>("no_loop_path", 1);
    pub_match_points = nh.advertise<sensor_msgs::PointCloud>("match_points", 1);

    // TODO petrlmat: abstract away threads, kill them on node death
    std::thread measurement_process = std::thread{&PoseGraphNodelet::process, this};
    measurement_process.detach();

    std::thread keyboard_command_process = std::thread{&PoseGraphNodelet::command, this};
    keyboard_command_process.detach();

    is_initialized_ = true;

    ROS_INFO("[%s]: initialized", node_name.c_str());
}
/*//}*/

/*//{ newSequence() */
void PoseGraphNodelet::newSequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}
/*//}*/

/*//{ callbackImage() */
void PoseGraphNodelet::callbackImage(const sensor_msgs::ImageConstPtr &image_msg)
{
    //ROS_INFO("image_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    image_buf.push(image_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 || image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! detect a new sequence!");
        newSequence();
    }
    last_image_time = image_msg->header.stamp.toSec();
}
/*//}*/

/*//{ callbackPoint() */
void PoseGraphNodelet::callbackPoint(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x, 
                                                     point_msg->points[i].y,
                                                     point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],
                                                     point_msg->channels[i].values[1]);
    }
    */
}
/*//}*/

/*//{ callbackPose() */
void PoseGraphNodelet::callbackPose(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    if(!LOOP_CLOSURE)
        return;
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}
/*//}*/

/*//{ callbackImuForward() */
void PoseGraphNodelet::callbackImuForward(const nav_msgs::Odometry::ConstPtr &forward_msg)
{

    if (!is_initialized_) 
    {
      return;
    }

    if (VISUALIZE_IMU_FORWARD)
    {
        Vector3d vio_t(forward_msg->pose.pose.position.x, forward_msg->pose.pose.position.y, forward_msg->pose.pose.position.z);
        Quaterniond vio_q;
        vio_q.w() = forward_msg->pose.pose.orientation.w;
        vio_q.x() = forward_msg->pose.pose.orientation.x;
        vio_q.y() = forward_msg->pose.pose.orientation.y;
        vio_q.z() = forward_msg->pose.pose.orientation.z;

        vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
        vio_q = posegraph.w_r_vio *  vio_q;

        vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
        vio_q = posegraph.r_drift * vio_q;

        Vector3d vio_t_cam;
        Quaterniond vio_q_cam;
        vio_t_cam = vio_t + vio_q * tic;
        vio_q_cam = vio_q * qic;        

        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, forward_msg->header);
    }
}
/*//}*/

/*//{ callbackReloRelativePose() */
void PoseGraphNodelet::callbackReloRelativePose(const nav_msgs::Odometry::ConstPtr &pose_msg)
{

    if (!is_initialized_) 
    {
      return;
    }

    Vector3d relative_t = Vector3d(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
    Quaterniond relative_q;
    relative_q.w() = pose_msg->pose.pose.orientation.w;
    relative_q.x() = pose_msg->pose.pose.orientation.x;
    relative_q.y() = pose_msg->pose.pose.orientation.y;
    relative_q.z() = pose_msg->pose.pose.orientation.z;
    double relative_yaw = pose_msg->twist.twist.linear.x;
    int index = pose_msg->twist.twist.linear.y;
    //printf("receive index %d \n", index );
    Eigen::Matrix<double, 8, 1 > loop_info;
    loop_info << relative_t.x(), relative_t.y(), relative_t.z(),
                 relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                 relative_yaw;
    posegraph.updateKeyFrameLoop(index, loop_info);

}
/*//}*/

/*//{ callbackVio() */
void PoseGraphNodelet::callbackVio(const nav_msgs::Odometry::ConstPtr &pose_msg)
{

    if (!is_initialized_) 
    {
      return;
    }

    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    if (!VISUALIZE_IMU_FORWARD)
    {
        cameraposevisual.reset();
        cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
        cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
    }

    odometry_buf.push(vio_t_cam);
    if (odometry_buf.size() > 10)
    {
        odometry_buf.pop();
    }

    visualization_msgs::Marker key_odometrys;
    key_odometrys.header = pose_msg->header;
    key_odometrys.header.frame_id = uav_name + "/vins_world";
    key_odometrys.ns = "key_odometrys";
    key_odometrys.type = visualization_msgs::Marker::SPHERE_LIST;
    key_odometrys.action = visualization_msgs::Marker::ADD;
    key_odometrys.pose.orientation.w = 1.0;
    key_odometrys.lifetime = ros::Duration();

    //static int key_odometrys_id = 0;
    key_odometrys.id = 0; //key_odometrys_id++;
    key_odometrys.scale.x = 0.1;
    key_odometrys.scale.y = 0.1;
    key_odometrys.scale.z = 0.1;
    key_odometrys.color.r = 1.0;
    key_odometrys.color.a = 1.0;

    for (unsigned int i = 0; i < odometry_buf.size(); i++)
    {
        geometry_msgs::Point pose_marker;
        Vector3d vio_t;
        vio_t = odometry_buf.front();
        odometry_buf.pop();
        pose_marker.x = vio_t.x();
        pose_marker.y = vio_t.y();
        pose_marker.z = vio_t.z();
        key_odometrys.points.push_back(pose_marker);
        odometry_buf.push(vio_t);
    }
    pub_key_odometrys.publish(key_odometrys);

    if (!LOOP_CLOSURE)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = pose_msg->header;
        pose_stamped.header.frame_id = uav_name + "/vins_world";
        pose_stamped.pose.position.x = vio_t.x();
        pose_stamped.pose.position.y = vio_t.y();
        pose_stamped.pose.position.z = vio_t.z();
        no_loop_path.header = pose_msg->header;
        no_loop_path.header.frame_id = uav_name + "/vins_world";
        no_loop_path.poses.push_back(pose_stamped);
        pub_vio_path.publish(no_loop_path);
    }
}
/*//}*/

/*//{ callbackExtrinsic() */
void PoseGraphNodelet::callbackExtrinsic(const nav_msgs::Odometry::ConstPtr &pose_msg)
{

    if (!is_initialized_) 
    {
      return;
    }

    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}
/*//}*/

/*//{ process() */
void PoseGraphNodelet::process()
{

    if (!is_initialized_) 
    {
      return;
    }

    if (!LOOP_CLOSURE)
        return;
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());
            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
            
            cv::Mat image = ptr->image;
            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence, FOCAL_LENGTH);   
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}
/*//}*/

/*//{ command() */
void PoseGraphNodelet::command()
{
    if (!LOOP_CLOSURE)
        return;
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            // printf("program shutting down...\n");
            // ros::shutdown();
        }
        if (c == 'n')
            newSequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}
/*//}*/

}
/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(vins_mono::PoseGraphNodelet, nodelet::Nodelet);
