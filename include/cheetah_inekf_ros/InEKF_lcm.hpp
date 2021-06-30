#pragma once


#include <mutex>
#include <thread>

// ROS related
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"
#include "inekf_msgs/State.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "KinematicsPublisher.hpp"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/imu_t.hpp"
#include "lcm-types/legcontrol_t.hpp"
#include "lcm-types/contact_t.hpp"
#include "lcm-types/simulator_t.hpp"
#include "lcm-types/leg_control_data_lcmt.hpp"
#include "lcm-types/microstrain_lcmt.hpp"
#include "lcm-types/simulator_lcmt.hpp"
#include "lcm-types/inekf_visualization_lcmt.hpp"

namespace cheetah_inekf_ros {
  template <unsigned int ENCODER_DIM>
class InEKF_lcm {
  public:
    InEKF_lcm() : nh_("~") {
      
      ROS_INFO("Cheetah_Lcm ready to initialize...."); 
	    
      seq_imu_data_ = 0;
      seq_joint_state_ = 0;
      seq_contact_ = 0;
      seq_sim_ = 0;
      nh_.param<double>("publish_rate", publish_rate_, 100000); 
      std::string joint_state_lcm_topic, imu_lcm_topic, contact_lcm_topic,  imu_frame_id, sim_lcm_topic ;
      nh_.param<std::string>("joint_state_lcm_topic", joint_state_lcm_topic, "joint_state_lcm");
      nh_.param<std::string>("contact_lcm_topic", contact_lcm_topic, "contact_lcm");
      nh_.param<std::string>("imu_lcm_topic", imu_lcm_topic, "imu_lcm");

      nh_.param<bool>("is_listening_sim", is_listening_sim_, true);
      nh_.param<std::string>("sim_lcm_topic", sim_lcm_topic, "simulator_state");
      nh_.param<double>("sim_contact_force_threshold", sim_contact_force_threshold_, 17.5);
      // subscribe to lcm TODO

      imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 200);
      joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 100);
      kinematics_publisher_ = nh_.advertise<inekf_msgs::KinematicsArray>("kinematics", 100);

      if (is_listening_sim_) {
        ROS_DEBUG("INEKF_LCM is listening simulator's lcm msg");
        sim_contact_publisher_ = nh_.advertise<inekf_msgs::ContactArray>("sim_contact", 20);
        sim_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("sim_pose", 50);
      } else
        contact_publisher_ = nh_.advertise<inekf_msgs::ContactArray>("contact", 20);

      double encoder_std, kinematic_prior_orientation_std, kinematic_prior_position_std;
      nh_.param<double>("encoder_std", encoder_std, 0.0174533); // 1 deg std
      nh_.param<double>("kinematic_prior_orientation_std", kinematic_prior_orientation_std, 0.174533); // 10 deg std
      nh_.param<double>("kinematic_prior_position_std", kinematic_prior_position_std, 0.05); // 5cm std

      cov_encoders_ = encoder_std*encoder_std*Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM>::Identity(); 
      cov_prior_ = Eigen::Matrix<double,6,6>::Identity();
      cov_prior_.block<3,3>(0,0) = kinematic_prior_orientation_std*kinematic_prior_orientation_std*Eigen::Matrix<double,3,3>::Identity();
      cov_prior_.block<3,3>(3,3) = kinematic_prior_position_std*kinematic_prior_position_std*Eigen::Matrix<double,3,3>::Identity();

      // publishing inekf state to lcm
      std::string inekf_state_topic;
      nh_.param<bool>("is_publishing_lcm", is_publishing_lcm_, true);
      nh_.param<std::string>("inekf_state_topic", inekf_state_topic, "/cheetah/inekf_state");
      
      if(is_publishing_lcm_){
        std::cout<<"inekf topic is: "<<inekf_state_topic<<std::endl;
        inEKF_state_subscriber_ = nh_.subscribe(inekf_state_topic, 1000, &InEKF_lcm::inekf_state_ros_callback, this);
        this->lcm_publishing_thread_ = std::thread([this]{this->lcm_publishing_thread();});
      }

      ROS_INFO("Cheetah_Lcm initialized."); 
    }

    void imu_lcm_callback2(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel_name,
                        const imu_t* msg);
    
    void joint_state_lcm_callback2(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const legcontrol_t* msg);

    void contact_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const contact_t* msg);

    
    void simulator_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                const std::string& channel_name,
                                const simulator_lcmt* msg);

    void imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                          const std::string& channel_name,
                          const microstrain_lcmt* msg);

    void joint_state_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                  const std::string& channel_name,
                                  const leg_control_data_lcmt* msg);

    void inekf_state_ros_callback(const inekf_msgs::State::ConstPtr &msg);

    // void inekf_kinematics_ros_callback(const inekf_msgs::KinematicsArray::ConstPtr &msg);
    inekf_visualization_lcmt get_inekf_lcm(){return inekf_lcm_;};

    // lcm publishing thread
    void lcm_publishing_thread(){
        // Loop and publish data
        ros::Rate loop_rate(publish_rate_);
        while(ros::ok()){
            // std::cout<<"inekf state lcm: "<<inekf_lcm_.x[0]<<", "<<inekf_lcm_.x[1]<<", "<<inekf_lcm_.x[2]<<std::endl;
            lcm_.publish("inekf_visualization",&inekf_lcm_);
            loop_rate.sleep();
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher contact_publisher_;
    ros::Publisher kinematics_publisher_;
    ros::Publisher sim_contact_publisher_;
    ros::Publisher sim_pose_publisher_;
    ros::Subscriber inEKF_state_subscriber_;

    Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM> cov_encoders_;
    Eigen::Matrix<double,6,6> cov_prior_;
    bool is_listening_sim_;
    double sim_contact_force_threshold_;
    bool is_publishing_lcm_;
    lcm::LCM lcm_;
    double publish_rate_;
    std::mutex inekf_lcm_mutex_;
    std::thread lcm_publishing_thread_;

  //std::string imu_lcm_topic, joint_state_lcm_topic, contact_lcm_topic;
  
    uint64_t seq_imu_data_;
    uint64_t seq_joint_state_;
    uint64_t seq_contact_;
    uint64_t seq_sim_;
    inekf_visualization_lcmt inekf_lcm_;
    
};

} // namespace mini_cheetah
