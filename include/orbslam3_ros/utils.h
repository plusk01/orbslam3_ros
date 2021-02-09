/**
 * @file orbslam3_ros.h
 * @brief ORB_SLAM3 ROS Wrapper
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 Februray 2021
 */

#pragma once

#include <fstream>
#include <iomanip>
#include <stdexcept>

#include <ros/ros.h>

#include <nav_msgs/Path.h>

namespace orbslam3 {
namespace utils {

  template <typename T>
  void getParamOrDie(const ros::NodeHandle& nh, const std::string& param, T& value)
  {
    if (!nh.getParam(param, value)) {
      ROS_ERROR_STREAM("Missing ROS parameter: " << param);
      ros::shutdown();
      throw std::runtime_error("Missing required ROS parameters");
    }
  }

  // --------------------------------------------------------------------------

  /**
   * @brief      Save nav_msgs/Path as a TUM trajectory file for post analysis
   *
   * @param[in]  fname    File to save as
   * @param[in]  pathmsg  A nav_msgs/Path containing trajectory to save
   */
  void saveTUMPath(const std::string& fname, const nav_msgs::Path& pathmsg)
  {
    std::ofstream f(fname);
    f << std::fixed;
    for (const auto& msg : pathmsg.poses) {
      f << std::setprecision(6) << msg.header.stamp.toSec() << " "
        << std::setprecision(7) << msg.pose.position.x << " "
        << msg.pose.position.y << " " << msg.pose.position.z << " "
        << msg.pose.orientation.x << " " << msg.pose.orientation.y << " "
        << msg.pose.orientation.z << " " << msg.pose.orientation.w << std::endl;
    }
    f.close();
  }

} // ns utils
} // ns orbslam3