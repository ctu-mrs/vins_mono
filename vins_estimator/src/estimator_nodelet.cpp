#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vins_estimator/estimator.h>
#include <vins_estimator/parameters.h>
#include <vins_estimator/utility/visualization.h>

namespace vins_mono
{

using namespace vins_estimator;

/*//{ class VinsEstimator */
class VinsEstimator : public nodelet::Nodelet {

public:

  ~VinsEstimator() {
    con.notify_one();
  };

  virtual void onInit();

private:

  Estimator estimator;

  bool is_initialized_ = false;

  std::condition_variable con;
  double current_time = -1;
  queue<sensor_msgs::ImuConstPtr> imu_buf;
  queue<sensor_msgs::PointCloudConstPtr> feature_buf;
  queue<sensor_msgs::PointCloudConstPtr> relo_buf;
  int sum_of_wait = 0;

  std::mutex m_buf;
  std::mutex m_state;
  std::mutex i_buf;
  std::mutex m_estimator;

  double latest_time;
  Eigen::Vector3d tmp_P;
  Eigen::Quaterniond tmp_Q;
  Eigen::Vector3d tmp_V;
  Eigen::Vector3d tmp_Ba;
  Eigen::Vector3d tmp_Bg;
  Eigen::Vector3d acc_0;
  Eigen::Vector3d gyr_0;
  bool init_feature = 0;
  bool init_imu = 1;
  double last_imu_t = 0;
  string uav_name = "";

  void predict(const sensor_msgs::ImuConstPtr &imu_msg);

  void update();

  std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> getMeasurements();

  ros::Subscriber sub_imu_; 
  void callbackImu(const sensor_msgs::ImuConstPtr &imu_msg);

  ros::Subscriber sub_features_; 
  void callbackFeatures(const sensor_msgs::PointCloudConstPtr &feature_msg);

  ros::Subscriber sub_restart_; 
  void callbackRestart(const std_msgs::BoolConstPtr &restart_msg);

  ros::Subscriber sub_relo_points_; 
  void callbackRelocalization(const sensor_msgs::PointCloudConstPtr &points_msg);

  void process();

};
/*//}*/

/*//{ onInit() */
void VinsEstimator::onInit()
{

    const std::string node_name("VinsEstimator");

    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    ROS_INFO("[%s]: Initializing", NODE_NAME.c_str());

    /* waits for the ROS to publish clock */
    ros::Time::waitForValid();

    nh.param<std::string>("uav_name", uav_name, "uav1");

    bool debug;
    nh.param<bool>("debug", debug, false);

    if (debug)
    {
    ROS_INFO("[%s]: debug: true", NODE_NAME.c_str());
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    }
    else
    {
    ROS_INFO("[%s]: debug: false", NODE_NAME.c_str());
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    }

    readParameters(nh);

    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_WARN("[%s]: EIGEN_DONT_PARALLELIZE", NODE_NAME.c_str());
#endif

    ROS_INFO("[%s]: waiting for image and imu...", NODE_NAME.c_str());

    registerPub(nh);

    sub_imu_ = nh.subscribe("imu_in", 1, &VinsEstimator::callbackImu, this, ros::TransportHints().tcpNoDelay());
    sub_features_ = nh.subscribe("feature_tracker/feature", 1, &VinsEstimator::callbackFeatures, this, ros::TransportHints().tcpNoDelay());
    sub_restart_ = nh.subscribe("feature_tracker/restart", 1, &VinsEstimator::callbackRestart, this, ros::TransportHints().tcpNoDelay());
    sub_relo_points_ = nh.subscribe("pose_graph/match_points", 1, &VinsEstimator::callbackRelocalization, this, ros::TransportHints().tcpNoDelay());

    std::thread measurement_process{&VinsEstimator::process, this};
    measurement_process.detach();

    ROS_INFO("[%s]: initialized", NODE_NAME.c_str());
    is_initialized_ = true;
}
/*//}*/

/*//{ predict() */
void VinsEstimator::predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}
/*//}*/

/*//{ update() */
void VinsEstimator::update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
    {
        predict(tmp_imu_buf.front());
    }

}
/*//}*/

/*//{ getMeasurements() */
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> VinsEstimator::getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
        {
            return measurements;
        }

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.back()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("[VinsEstimator]: Waiting for imu, only should happen at the beginning. IMU frequency is too low.");
            sum_of_wait++;
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.back()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("[VinsEstimator]: Throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }

        // note petrlmat: changed from the og code: we want to always use the newest image to keep delay low
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.back();
        while (!feature_buf.empty()) {
          feature_buf.pop();
        }

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
        {
            ROS_WARN("[VinsEstimator]: No imu msgs between two images");
        }
        measurements.emplace_back(IMUs, img_msg);
    }
    // This part of code can be never reached
    return measurements;
}
/*//}*/

/*//{ callbackImu() */
void VinsEstimator::callbackImu(const sensor_msgs::ImuConstPtr &imu_msg)
{

    if (!is_initialized_) {
      return;
    }

    ROS_INFO_ONCE("[%s]: got imu message", NODE_NAME.c_str());

    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    /* last_imu_t = imu_msg->header.stamp.toSec(); */

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        // Publish only after initialization
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            const double rx = imu_msg->angular_velocity.x;
            const double ry = imu_msg->angular_velocity.y;
            const double rz = imu_msg->angular_velocity.z;
            Eigen::Vector3d ang_vel{rx, ry, rz};
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, ang_vel - tmp_Bg, header);
        }
    }
}
/*//}*/

/*//{ callbackFeatures() */
void VinsEstimator::callbackFeatures(const sensor_msgs::PointCloudConstPtr &feature_msg)
{

    if (!is_initialized_) {
      return;
    }

    ROS_INFO_ONCE("[%s]: got feature message", NODE_NAME.c_str());

    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }

    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one();
}
/*//}*/

/*//{ callbackRestart() */
void VinsEstimator::callbackRestart(const std_msgs::BoolConstPtr &restart_msg)
{

    if (!is_initialized_) {
      return;
    }

    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState();
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}
/*//}*/

/*//{ callbackRelocalization() */
void VinsEstimator::callbackRelocalization(const sensor_msgs::PointCloudConstPtr &points_msg)
{

    if (!is_initialized_) {
      return;
    }

    //printf("relocalization callback! \n");
    m_buf.lock();
    relo_buf.push(points_msg);
    m_buf.unlock();
}
/*//}*/

/*//{ process() */
// thread: visual-inertial odometry
void VinsEstimator::process()
{
    // run this thread while ros is alive
    while (ros::ok())
    {
        std::string imu_frame_id;
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return ((measurements = getMeasurements()).size() != 0 || ros::isShuttingDown());
                 });
        lk.unlock();
        m_estimator.lock();
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                imu_frame_id = imu_msg->header.frame_id;
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                // IMU msg is older than img msg
                // current_time is the time of the latest processed msg, be it IMU or img
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    /* printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz); */

                }
                // IMU msg is newer than img msg
                else
                {
                    // dt_1: time from last processed msg to last image
                    double dt_1 = img_t - current_time;
                    // dt_2: time from last img to last imu
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    /* printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz); */
                }
            }
            // set relocalization frame
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            estimator.processImage(image, img_msg->header);

            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t);
            std_msgs::Header header = img_msg->header;
            header.frame_id = VINS_WORLD_FRAME_ID;

            Eigen::Vector3d ang_vel{rx, ry, rz};
            pubOdometry(estimator, ang_vel, header, imu_frame_id);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
            pubDiagnostics(estimator, header, whole_t);
            pubBias(estimator, header, imu_frame_id);
            if (relo_msg != NULL)
            {
                pubRelocalization(estimator);
            }
            //ROS_ERROR("end: %f, at %f", img_msg->header.stamp.toSec(), ros::Time::now().toSec());
        }
        m_estimator.unlock();
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
        {
            update();
        }
        m_state.unlock();
        m_buf.unlock();
    }
    ROS_WARN("[%s]: exiting process thread", NODE_NAME.c_str());
}
/*//}*/

}

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(vins_mono::VinsEstimator, nodelet::Nodelet);
