/**
 * @file orbslam3_ros.h
 * @brief ORB_SLAM3 ROS Wrapper
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 Februray 2021
 */

#pragma once

#include <memory>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

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
    ros::Subscriber sub_gt_;
    ros::Publisher pub_truepath_, pub_path_, pub_pose_;
    ros::ServiceServer srv_save_;

    message_filters::Subscriber<sensor_msgs::Image> smf_img_, smf_depth_;

    /// \brief Synchronization of RGBD topics
    using SyncPolicyRGBD = message_filters::sync_policies::ApproximateTime<
                          sensor_msgs::Image, sensor_msgs::Image>;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicyRGBD>> syncrgbd_;


    // \brief ORB_SLAM3 system
    int sensor_; ///< mono=0, stereo=1, rgbd=2, imu+mono=3, imu+stereo=4
    std::unique_ptr<ORB_SLAM3::System> slam_;

    // \brief Tracking results
    geometry_msgs::PoseStamped posemsg_;
    nav_msgs::Path pathmsg_;

    nav_msgs::Path truepathmsg_; ///< collection of true poses of body w.r.t world
    Eigen::Affine3d Toww_; ///< {world w.r.t original world}
    Eigen::Affine3d Twm_; ///< map (cam start) w.r.t world (ENU)
    Eigen::Affine3d Tcb_; ///< body (flu) w.r.t camera

    void init_system();
    void handle_tracking_results(const cv::Mat& _Tcm);

    // \brief ROS Callbacks
    void truepose_cb(const geometry_msgs::PoseStamped& msg);
    void img1_cb(const sensor_msgs::ImageConstPtr& msg);
    void rgbd_cb(const sensor_msgs::ImageConstPtr& imgmsg, const sensor_msgs::ImageConstPtr& depthmsg);
    bool save_cb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  };

} // ns orbslam3
