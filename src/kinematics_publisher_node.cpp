#include "cheetah_inekf_ros/KinematicsPublisher.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematics_publisher_node");
  ros::NodeHandle n;
  // cheetah has 12 degrees of freedom
  cheetah_inekf_ros::KinematicsPublisher<12> kinematics_publisher_node(n);
  ros::spin();
  return 0;
}
