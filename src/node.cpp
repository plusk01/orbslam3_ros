/**
 * @file node.cpp
 * @brief ROS entry point for ORB_SLAM3
 * @author Parker Lusk <plusk@mit.edu>
 * @date 9 February 2021
 */

#include <ros/ros.h>

#include "orbslam3_ros/orbslam3_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orbslam3");
  ros::NodeHandle nhtopics("");
  ros::NodeHandle nhparams("~");
  orbslam3::ROSWrapper wrappter(nhtopics, nhparams);
  ros::spin();
  return 0;
}