/**
 * @file orbslam3_ros.h
 * @brief ORB_SLAM3 ROS Wrapper
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 Februray 2021
 */

#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <System.h>

namespace orbslam3 {

  class ROSWrapper
  {
  public:
    ROSWrapper(const ros::NodeHandle& nh, const ros::NodeHandle& nhp);
    ~ROSWrapper() = default;

  private:
    ros::NodeHandle nh_, nhp_;
    ros::Subscriber sub_img1_, sub_img2_; ///< use img1 for mono

    /// \brief ORB_SLAM3 system
    std::unique_ptr<ORB_SLAM3::System> slam_;

    void init_system();

    // \brief ROS Callbacks
    void img1_cb(const sensor_msgs::ImageConstPtr& msg);

  };

} // ns orbslam3
