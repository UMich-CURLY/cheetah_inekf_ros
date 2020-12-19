#include "cheetah_inekf_ros/InEKF_lcm.hpp"

namespace cheetah_inekf_ros {
  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& channel_name,
                                   const microstrain_lcmt* msg) {
  
    seq_imu_data_++;

    sensor_msgs::Imu imu_msg;
    imu_msg.header.seq = seq_imu_data_;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "/cheetah/imu";
    imu_msg.orientation.w = msg->quat[0];
    imu_msg.orientation.x = msg->quat[1];
    imu_msg.orientation.y = msg->quat[2];
    imu_msg.orientation.z = msg->quat[3];
    imu_msg.angular_velocity.x = msg->omega[0];
    imu_msg.angular_velocity.y = msg->omega[1];
    imu_msg.angular_velocity.z = msg->omega[2];
    imu_msg.linear_acceleration.x = msg->acc[0];
    imu_msg.linear_acceleration.y = msg->acc[1];
    imu_msg.linear_acceleration.z = msg->acc[2];
    imu_publisher_.publish(imu_msg);                            
  }

  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::joint_state_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                           const std::string& channel_name,
                                           const leg_control_data_lcmt* msg) {
    seq_joint_state_++;

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.seq = seq_joint_state_;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.header.frame_id = "/cheetah/joint_state";
    std::vector<double> joint_position(msg->q, msg->q + sizeof(msg->q)/sizeof(msg->q[0]));
    std::vector<double> joint_velocity(msg->qd, msg->qd + sizeof(msg->qd)/sizeof(msg->qd[0]));
    std::vector<double> joint_effort(msg->tau_est, msg->tau_est + sizeof(msg->tau_est)/sizeof(msg->tau_est[0]));
    Eigen::Matrix<double, ENCODER_DIM, 1> encoder_pos;
    for (int j = 0; j < ENCODER_DIM; j++)
      encoder_pos(j) = joint_position[j];
    joint_state_msg.position = joint_position;
    joint_state_msg.velocity = joint_velocity;
    joint_state_msg.effort = joint_effort;
    joint_state_publisher_.publish(joint_state_msg);

    std_msgs::Header header;
    inekf_msgs::KinematicsArray kinematics_arr = KinematicsPublisher<ENCODER_DIM>::callback_handler(header, encoder_pos, cov_encoders_, cov_prior_);
    kinematics_arr.header.seq = seq_joint_state_;
    kinematics_arr.header.stamp = joint_state_msg.header.stamp;
    kinematics_arr.header.frame_id =  "/cheetah/imu";
    kinematics_publisher_.publish(kinematics_arr);
    
  }

  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::contact_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& channel_name,
                                       const wbc_test_data_t* msg) {
    seq_contact_++;

    inekf_msgs::ContactArray contact_msg;
    contact_msg.header.seq = seq_contact_;
    contact_msg.header.stamp = ros::Time::now();
    contact_msg.header.frame_id = "/cheetah/contact";

    std::vector<inekf_msgs::Contact> contacts;

    for (int i = 0; i < 4; i++)
    {
      inekf_msgs::Contact ct;
      ct.id = i;
      ct.indicator =  msg->contact_est[i] > 0;
      contacts.push_back(ct);
    }
    contact_msg.contacts = contacts;
    contact_publisher_.publish(contact_msg);
  }



} // mini_cheetah

template class cheetah_inekf_ros::KinematicsPublisher<12>;
template class cheetah_inekf_ros::InEKF_lcm<12>;
