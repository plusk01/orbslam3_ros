/**
 * @file orbslam3_ros.h
 * @brief ORB_SLAM3 ROS Wrapper
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 Februray 2021
 */

#pragma once

#include <stdexcept>

#include <ros/ros.h>

namespace orbslam3 {
namespace utils {

  template <typename T>
  void getParamOrDie(const ros::NodeHandle& nh, const std::string& param, T& value) {
    if (!nh.getParam(param, value)) {
      ROS_ERROR_STREAM("Missing ROS parameter: " << param);
      ros::shutdown();
      throw std::runtime_error("Missing required ROS parameters");
    }
  }

} // ns utils
} // ns orbslam3