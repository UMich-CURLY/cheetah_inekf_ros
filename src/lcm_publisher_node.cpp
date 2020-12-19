#include "cheetah_inekf_ros/KinematicsPublisher.hpp"
#include "cheetah_inekf_ros/InEKF_lcm.hpp"

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "InEKF_lcm");
  ros::NodeHandle n;
  // cheetah has 12 degrees of freedom
  cheetah_inekf_ros::InEKF_lcm<12> lcm_publisher_node;
  ros::spin();
  return 0;
}
