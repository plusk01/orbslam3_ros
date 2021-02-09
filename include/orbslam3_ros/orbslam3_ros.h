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
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

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
    ros::Publisher pub_path_, pub_pose_;

    // \brief ORB_SLAM3 system
    std::unique_ptr<ORB_SLAM3::System> slam_;

    // \brief Tracking results
    geometry_msgs::PoseStamped posemsg_;
    nav_msgs::Path pathmsg_;

    Eigen::Affine3d Twm_;
    Eigen::Affine3d Tcb_;

    void init_system();
    void handle_tracking_results(const cv::Mat& _Tcm);

    // \brief ROS Callbacks
    void img1_cb(const sensor_msgs::ImageConstPtr& msg);

  };

} // ns orbslam3
