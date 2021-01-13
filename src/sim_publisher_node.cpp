#include "cheetah_inekf_ros/KinematicsPublisher.hpp"
#include "cheetah_inekf_ros/InEKF_lcm.hpp"

#include "ros/ros.h"
#include "ros/console.h"
#include <lcm/lcm-cpp.hpp>
#include "cheetah_inekf_ros/lcm-types/inekf_visualization_lcmt.hpp"
#include "cheetah_inekf_ros/lcm-types/imu_t.hpp"

int main(int argc, char **argv) {
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
  ROS_DEBUG_STREAM("start new lcm to ros node...");

  lcm::LCM lcm;
  if(!lcm.good()) {
    ROS_ERROR_STREAM("LCM init failed.");
    return -1;
  }
  
  
  ros::init(argc, argv, "InEKF_lcm");
  ros::NodeHandle n;

  std::cout<<"starting a new node...\n";
  // cheetah has 12 degrees of freedom
  cheetah_inekf_ros::InEKF_lcm<12> sim_lcm_publisher_node;
  
  lcm.subscribe("microstrain", &cheetah_inekf_ros::InEKF_lcm<12>::imu_lcm_callback, &sim_lcm_publisher_node);
  lcm.subscribe("leg_control_data", &cheetah_inekf_ros::InEKF_lcm<12>::joint_state_lcm_callback, &sim_lcm_publisher_node);
  lcm.subscribe("simulator_state", &cheetah_inekf_ros::InEKF_lcm<12>::simulator_lcm_callback, &sim_lcm_publisher_node);
  

  
  
  ROS_DEBUG_STREAM("start spinning...");  
  while( ( lcm.handle() == 0 ) && (ros::ok()) ) //
    ros::spinOnce();
  return 0;
}
