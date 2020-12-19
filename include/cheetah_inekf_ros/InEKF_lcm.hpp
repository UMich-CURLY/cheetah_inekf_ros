#pragma once

// ROS related
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"
#include "KinematicsPublisher.hpp"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/microstrain_lcmt.hpp"
#include "lcm-types/leg_control_data_lcmt.hpp"
#include "lcm-types/wbc_test_data_t.hpp"

namespace cheetah_inekf_ros {
  template <unsigned int ENCODER_DIM>
class InEKF_lcm {
  public:
    InEKF_lcm() : nh_() {
      seq_imu_data_ = 0;
      seq_joint_state_ = 0;
      seq_contact_ = 0;

      std::string joint_state_lcm_topic, imu_lcm_topic, contact_lcm_topic,  imu_frame_id ;
      nh_.param<std::string>("joint_state_lcm_topic", joint_state_lcm_topic, "joint_state_lcm");
      nh_.param<std::string>("contact_lcm_topic", contact_lcm_topic, "contact_lcm");
      nh_.param<std::string>("imu_lcm_topic", imu_lcm_topic, "imu_lcm");
      // subscribe to lcm TODO

      imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
      joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
      kinematics_publisher_ = nh_.advertise<inekf_msgs::KinematicsArray>("kinematics", 10);
      contact_publisher_ = nh_.advertise<inekf_msgs::ContactArray>("contact", 10);

      double encoder_std, kinematic_prior_orientation_std, kinematic_prior_position_std;
      nh_.param<double>("encoder_std", encoder_std, 0.0174533); // 1 deg std
      nh_.param<double>("kinematic_prior_orientation_std", kinematic_prior_orientation_std, 0.174533); // 10 deg std
      nh_.param<double>("kinematic_prior_position_std", kinematic_prior_position_std, 0.05); // 5cm std

      cov_encoders_ = encoder_std*encoder_std*Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM>::Identity(); 
      cov_prior_ = Eigen::Matrix<double,6,6>::Identity();
      cov_prior_.block<3,3>(0,0) = kinematic_prior_orientation_std*kinematic_prior_orientation_std*Eigen::Matrix<double,3,3>::Identity();
      cov_prior_.block<3,3>(3,3) = kinematic_prior_position_std*kinematic_prior_position_std*Eigen::Matrix<double,3,3>::Identity();
      
    }

    void imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel_name,
                        const microstrain_lcmt* msg);
    
    void joint_state_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const leg_control_data_lcmt* msg);

    void contact_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const wbc_test_data_t* msg);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher contact_publisher_;
    ros::Publisher kinematics_publisher_;

    Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM> cov_encoders_;
    Eigen::Matrix<double,6,6> cov_prior_;

  //std::string imu_lcm_topic, joint_state_lcm_topic, contact_lcm_topic;
  
    uint64_t seq_imu_data_;
    uint64_t seq_joint_state_;
    uint64_t seq_contact_;


};

} // namespace mini_cheetah
