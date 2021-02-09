/**
 * @file orbslam3_ros.cpp
 * @brief ORB_SLAM3 ROS Wrapper
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 Februray 2021
 */

#include "orbslam3_ros/orbslam3_ros.h"
#include "orbslam3_ros/utils.h"

#include <cv_bridge/cv_bridge.h>

namespace orbslam3 {

ROSWrapper::ROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nhp)
: nh_(nh), nhp_(nhp)
{
  init_system();

  //
  // ROS communication
  //
  
  sub_img1_ = nh_.subscribe("img1", 1, &ROSWrapper::img1_cb, this);
  // sub_img2_ = nh_.subscribe("img2", 1, &ROSWrapper::img2_cb, this);
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

  slam_->TrackMonocular(image, msg->header.stamp.toSec());

}

} // ns orbslam3