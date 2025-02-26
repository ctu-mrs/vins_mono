#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include <vins_estimator/utility/CameraPoseVisualization.h>
#include <vins_estimator/estimator.h>
#include <vins_estimator/parameters.h>

#include <vins_mono_vins_estimator/Diagnostics.h>

namespace vins_mono {
  namespace vins_estimator {

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path, pub_pose;
extern ros::Publisher pub_cloud, pub_map;
extern ros::Publisher pub_key_poses;
extern ros::Publisher pub_ref_pose, pub_cur_pose;
extern ros::Publisher pub_key;
extern nav_msgs::Path path;
extern ros::Publisher pub_pose_graph;
extern ros::Publisher pub_diagnostics;
extern ros::Publisher pub_bias_acc;
extern ros::Publisher pub_bias_gyro;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);

void pubLatestOdometry(const Eigen::Vector3d &P, const Eigen::Quaterniond &Q, const Eigen::Vector3d &V, const Eigen::Vector3d &ang_vel, const std_msgs::Header &imu_header);

void printStatistics(const Estimator &estimator, double t);

void pubOdometry(const Estimator &estimator, const Eigen::Vector3d &ang_vel, const std_msgs::Header &header, const std::string &child_frame_id);

void pubInitialGuess(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyPoses(const Estimator &estimator, const std_msgs::Header &header);

void pubCameraPose(const Estimator &estimator, const std_msgs::Header &header);

void pubPointCloud(const Estimator &estimator, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);

void pubKeyframe(const Estimator &estimator);

void pubRelocalization(const Estimator &estimator);

void pubDiagnostics(const Estimator &estimator, const std_msgs::Header &header, const double t_total);

void pubBias(const Estimator &estimator, const std_msgs::Header &header, const std::string &imu_frame_id);

  }
}
