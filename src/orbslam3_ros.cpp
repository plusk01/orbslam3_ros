/**
 * @file orbslam3_ros.cpp
 * @brief ORB_SLAM3 ROS Wrapper
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 Februray 2021
 */

#include "orbslam3_ros/orbslam3_ros.h"
#include "orbslam3_ros/utils.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/eigen.hpp>

namespace orbslam3 {

ROSWrapper::ROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp)
{
  init_system();

  // rotation of body (flu) w.r.t camera
  Tcb_.setIdentity();
  Eigen::Quaterniond qcb;
  qcb = Eigen::AngleAxisd(0      , Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd( M_PI/2, Eigen::Vector3d::UnitX());
  Tcb_.linear() = qcb.toRotationMatrix();

  // rotation of camera start pose (map origin) w.r.t ENU world
  // This is simply Tcb_.inverse()
  Twm_.setIdentity();
  Eigen::Quaterniond qwm;
  qwm = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0      , Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());
  Twm_.linear() = qwm.toRotationMatrix();

  //
  // ROS communication
  //
  
  posemsg_.header.frame_id = "world";
  pathmsg_.header.frame_id = posemsg_.header.frame_id;
  truepathmsg_.header.frame_id = posemsg_.header.frame_id;

  if (sensor_ == 0) { // mono

    sub_img1_ = nh_.subscribe("img1", 1, &ROSWrapper::img1_cb, this);

  } else if (sensor_ == 1) { // stereo
    // sub_img2_ = nh_.subscribe("img2", 1, &ROSWrapper::img2_cb, this);
    smf_img1_.subscribe(nh_, "img1", 1);
    smf_img2_.subscribe(nh_, "img2", 1);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100),
      smf_img1_, smf_img2_));
    sync_->registerCallback(&ROSWrapper::stereo_cb, this);
  } else if (sensor_ == 2) { // rgbd

    smf_img1_.subscribe(nh_, "img1", 1);
    smf_depth_.subscribe(nh_, "depth", 1);
    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(100),
      smf_img1_, smf_depth_));
    sync_->registerCallback(&ROSWrapper::rgbd_cb, this);

  } else if (sensor_ == 3) { // imu+mono

    sub_img1_ = nh_.subscribe("img1", 1, &ROSWrapper::img1_cb, this);
    sub_imu_ = nh_.subscribe("imu", 1000, &ROSWrapper::imu_cb, this);

  } else if (sensor_ == 4) { // imu+stereo
    // sub_img2_ = nh_.subscribe("img2", 1, &ROSWrapper::img2_cb, this);
  }
  use_imu_ = (sensor_ == 3 || sensor_ == 4);

  sub_gt_ = nh_.subscribe("truepose", 1, &ROSWrapper::truepose_cb, this);

  pub_truepath_ = nh_.advertise<nav_msgs::Path>("truepath", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);

  srv_save_ = nh_.advertiseService("save_traj_tum", &ROSWrapper::save_cb, this);

  if (sensor_ == 3) {
    slamthread_ = std::thread{&ROSWrapper::slamsync, this};
  }
}

// ----------------------------------------------------------------------------

ROSWrapper::~ROSWrapper()
{
  if (slamthread_.joinable()) slamthread_.join();
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void ROSWrapper::init_system()
{
  std::string vocfile, settingsfile;
  utils::getParamOrDie(nhp_, "vocab_file", vocfile);
  utils::getParamOrDie(nhp_, "settings_file", settingsfile);
  utils::getParamOrDie(nhp_, "sensor", sensor_);

  // initialize the ORB_SLAM3 system
  static constexpr bool visualize = true;
  const ORB_SLAM3::System::eSensor stype = static_cast<ORB_SLAM3::System::eSensor>(sensor_);
  slam_.reset(new ORB_SLAM3::System(vocfile, settingsfile, stype, visualize));
}

// ----------------------------------------------------------------------------

void ROSWrapper::handle_tracking_results(const cv::Mat& _Tcm)
{
  // if empty, tracking failed this iteration --> skip
  if (_Tcm.empty()) return;

  Eigen::Matrix4d tmp;
  cv::cv2eigen(_Tcm, tmp);
  Eigen::Affine3d Tcm;
  Tcm.matrix() = tmp;

  Eigen::Affine3d Twb = Twm_ * Tcm.inverse() * Tcb_;

  posemsg_.header.stamp = ros::Time::now();
  posemsg_.pose = tf2::toMsg(Twb);

  pathmsg_.header.stamp = posemsg_.header.stamp;
  pathmsg_.poses.push_back(posemsg_);

  pub_pose_.publish(posemsg_);
  pub_path_.publish(pathmsg_);
}

// ----------------------------------------------------------------------------

void ROSWrapper::slamsync()
{
  while (ros::ok()) {

    if (!buf_img1_.empty()) {
      const double t_img = buf_img1_.front()->header.stamp.toSec();

      // wait until we have IMUs to process
      if (use_imu_ && buf_imu_.empty()) continue;

      // wait until we've received all IMUs preceding this image
      if (use_imu_ && t_img > buf_imu_.back()->header.stamp.toSec()) continue;

      cv::Mat img1;
      {
        std::lock_guard<std::mutex> lock(mtx_img1_);
        img1 = unwrap_image(buf_img1_.front());
        buf_img1_.pop();
      }

      std::vector<ORB_SLAM3::IMU::Point> imus;
      if (use_imu_) {
        std::lock_guard<std::mutex> lock(mtx_imu_);
        // while there are IMUs preceding this image, get them
        while (!buf_imu_.empty() && buf_imu_.front()->header.stamp.toSec() <= t_img) {
          const auto imumsg = buf_imu_.front();
          cv::Point3f acc(imumsg->linear_acceleration.x, imumsg->linear_acceleration.y, imumsg->linear_acceleration.z);
          cv::Point3f gyr(imumsg->angular_velocity.x, imumsg->angular_velocity.y, imumsg->angular_velocity.z);
          imus.emplace_back(acc, gyr, imumsg->header.stamp.toSec());
          buf_imu_.pop();
        }
      }

      // ROS_WARN_STREAM("num imus: " << imus.size() << " " << t_img);

      cv::Mat Tcm;
      if (sensor_ == 0 || sensor_ == 3) {
        Tcm = slam_->TrackMonocular(img1, t_img, imus);
      }

      handle_tracking_results(Tcm);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

// ----------------------------------------------------------------------------

cv::Mat ROSWrapper::unwrap_image(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg)->image;
  } catch (cv_bridge::Exception& e) {
    // ROS_ERROR("Could not convert from '%s' to 'mono8'.", _img->encoding.c_str());
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return {};
  }
  // todo: don't clone
  return image.clone();
}

// ----------------------------------------------------------------------------
// ROS Callbacks
// ----------------------------------------------------------------------------

void ROSWrapper::truepose_cb(const geometry_msgs::PoseStamped& msg)
{
  static bool first = true;

  // capture offset of pose w.r.t world on first msg
  if (first) {
    // first {truepose w.r.t original world} == {world w.r.t original world}
    tf2::fromMsg(msg.pose, Toww_);
    first = false;
  }

  Eigen::Affine3d Towp;
  tf2::fromMsg(msg.pose, Towp);

  // transform {truepose w.r.t original world} to {truepose w.r.t world}
  Eigen::Affine3d Twp = Toww_.inverse() * Towp;

  geometry_msgs::PoseStamped posemsg;
  posemsg.header = truepathmsg_.header;
  posemsg.header.stamp = ros::Time::now();
  posemsg.pose = tf2::toMsg(Twp);

  truepathmsg_.header.stamp = ros::Time::now();
  truepathmsg_.poses.push_back(posemsg);

  pub_truepath_.publish(truepathmsg_);
}

// ----------------------------------------------------------------------------

// void ROSWrapper::truepose_cb(const geometry_msgs::PointStamped& msg)
// {
//   geometry_msgs::PoseStamped posemsg;
//   posemsg.header = msg.header;
//   posemsg.pose.orientation.w = 1.0;
//   posemsg.pose.position = msg.point;
//   truepose_cb(posemsg);
// }

// ----------------------------------------------------------------------------

void ROSWrapper::img1_cb(const sensor_msgs::ImageConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_img1_);
  // if (!buf_img1_.empty()) buf_img1_.pop();
  buf_img1_.push(msg);

  // cv::Mat image;
  // try {
  //   image = cv_bridge::toCvShare(msg)->image;
  // } catch (cv_bridge::Exception& e) {
  //   // ROS_ERROR("Could not convert from '%s' to 'mono8'.", _img->encoding.c_str());
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // // returns the pose of the map w.r.t camera
  // ROS_WARN_STREAM("num imus: " << imus_.size());
  // // cv::Mat Tcm = slam_->TrackMonocular(image, msg->header.stamp.toSec(), imus_);
  // imus_.clear();

  // // handle_tracking_results(Tcm);
}

// ----------------------------------------------------------------------------

void ROSWrapper::imu_cb(const sensor_msgs::ImuConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mtx_imu_);
  buf_imu_.push(msg);

  // double t = msg->header.stamp.toSec();
  // cv::Point3f acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  // cv::Point3f gyr(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  // imus_.emplace_back(acc, gyr, t);
}

// ----------------------------------------------------------------------------

void ROSWrapper::rgbd_cb(const sensor_msgs::ImageConstPtr& imgmsg,
  const sensor_msgs::ImageConstPtr& depthmsg)
{
  // cv::Mat rgb;
  // try {
  //   rgb = cv_bridge::toCvShare(imgmsg)->image;
  // } catch (cv_bridge::Exception& e) {
  //   // ROS_ERROR("Could not convert from '%s' to 'mono8'.", _img->encoding.c_str());
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // cv::Mat d;
  // try {
  //   d = cv_bridge::toCvShare(depthmsg)->image;
  // } catch (cv_bridge::Exception& e) {
  //   // ROS_ERROR("Could not convert from '%s' to 'mono8'.", _img->encoding.c_str());
  //   ROS_ERROR("cv_bridge exception: %s", e.what());
  //   return;
  // }

  // returns the pose of the map w.r.t camera
  // cv::Mat Tcm = slam_->TrackRGBD(rgb, d, imgmsg->header.stamp.toSec());

  // handle_tracking_results(Tcm);
}

// ----------------------------------------------------------------------------

void ROSWrapper::stereo_cb(const sensor_msgs::ImageConstPtr& leftmsg,
  const sensor_msgs::ImageConstPtr& rightmsg)
{
  cv::Mat left = unwrap_image(leftmsg);
  cv::Mat right = unwrap_image(rightmsg);

  // returns the pose of the map w.r.t camera
  cv::Mat Tcm = slam_->TrackStereo(left, right, leftmsg->header.stamp.toSec());

  handle_tracking_results(Tcm);
}

// ----------------------------------------------------------------------------

bool ROSWrapper::save_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  const std::string fname_orbslam = "orbslam3_traj_tum.txt";
  const std::string fname_truth = "truth_traj_tum.txt";

  // save paths as TUM files for analysis
  utils::saveTUMPath(fname_orbslam, pathmsg_);
  utils::saveTUMPath(fname_truth, truepathmsg_);

  res.success = true;
  res.message = "Saved to ~/.ros/" + fname_orbslam + " and ~/.ros/" + fname_truth;
  return true;
}

} // ns orbslam3