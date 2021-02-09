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

  sub_img1_ = nh_.subscribe("img1", 1, &ROSWrapper::img1_cb, this);
  // sub_img2_ = nh_.subscribe("img2", 1, &ROSWrapper::img2_cb, this);

  pub_path_ = nh_.advertise<nav_msgs::Path>("path", 1);
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void ROSWrapper::init_system()
{
  std::string vocfile, settingsfile;
  utils::getParamOrDie(nhp_, "vocab_file", vocfile);
  utils::getParamOrDie(nhp_, "settings_file", settingsfile);
  int sensor;
  utils::getParamOrDie(nhp_, "sensor", sensor);

  // initialize the ORB_SLAM3 system
  static constexpr bool visualize = true;
  const ORB_SLAM3::System::eSensor stype = static_cast<ORB_SLAM3::System::eSensor>(sensor);
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
// ROS Callbacks
// ----------------------------------------------------------------------------

void ROSWrapper::img1_cb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image;
  try {
    image = cv_bridge::toCvShare(msg)->image;
  } catch (cv_bridge::Exception& e) {
    // ROS_ERROR("Could not convert from '%s' to 'mono8'.", _img->encoding.c_str());
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // returns the pose of the map w.r.t camera
  cv::Mat Tcm = slam_->TrackMonocular(image, msg->header.stamp.toSec());

  handle_tracking_results(Tcm);
}

} // ns orbslam3