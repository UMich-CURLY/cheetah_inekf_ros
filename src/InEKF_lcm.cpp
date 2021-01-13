#include "cheetah_inekf_ros/InEKF_lcm.hpp"
#include <vector>
#include <algorithm>
namespace cheetah_inekf_ros {
  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::imu_lcm_callback2(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& channel_name,
                                   const imu_t* msg) {
    ROS_DEBUG_STREAM("Receive new imu msg");
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
  void InEKF_lcm<ENCODER_DIM>::joint_state_lcm_callback2(const lcm::ReceiveBuffer* rbuf,
                                                        const std::string& channel_name,
                                                        const legcontrol_t* msg) {
    ROS_DEBUG_STREAM("Receive new joint_state msg");    
    seq_joint_state_++;

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.seq = seq_joint_state_;
    joint_state_msg.header.stamp = ros::Time::now();
    //joint_state_msg.header.frame_id = "/cheetah/joint_state";
    std::vector<double> joint_position(msg->q.begin(), msg->q.end() );
    std::vector<double> joint_velocity(msg->qd.begin(), msg->qd.end() );
    std::vector<double> joint_effort(msg->tau_est.begin(), msg->tau_est.end() );
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
  void InEKF_lcm<ENCODER_DIM>::imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                                const std::string& channel_name,
                                                const microstrain_lcmt* msg) {
    ROS_DEBUG_STREAM("Receive new microstrain msg");
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
    ROS_DEBUG_STREAM("Receive new lcm joint_state msg");    
    seq_joint_state_++;

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.seq = seq_joint_state_;
    joint_state_msg.header.stamp = ros::Time::now();
    //joint_state_msg.header.frame_id = "/cheetah/joint_state";
    std::vector<double> joint_position(ENCODER_DIM);
    std::copy(msg->q, msg->q+ENCODER_DIM, joint_position.begin()  );
    std::vector<double> joint_velocity(ENCODER_DIM);
    std::copy(msg->qd, msg->qd+ENCODER_DIM, joint_velocity.begin() );
    std::vector<double> joint_effort(ENCODER_DIM);
    std::copy(msg->tau_est, msg->tau_est+ ENCODER_DIM, joint_effort.begin() );
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
                                       const contact_t* msg) {
    ROS_DEBUG_STREAM("Receive new contact msg");        
    seq_contact_++;

    inekf_msgs::ContactArray contact_msg;
    contact_msg.header.seq = seq_contact_;
    contact_msg.header.stamp = ros::Time::now();
    //contact_msg.header.frame_id = "/cheetah/contact";

    std::vector<inekf_msgs::Contact> contacts;

    for (int i = 0; i < 4; i++)
    {
      inekf_msgs::Contact ct;
      ct.id = i;
      ct.indicator =  msg->contact[i] > 0;
      contacts.push_back(ct);
    }
    contact_msg.contacts = contacts;
    contact_publisher_.publish(contact_msg);
  }

  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::simulator_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                                      const std::string& channel_name,
                                                      const simulator_lcmt* msg) {
    ROS_DEBUG_STREAM("Receive new simulator msg");        
    seq_sim_++;

    inekf_msgs::ContactArray contact_msg;
    contact_msg.header.seq = seq_sim_;
    contact_msg.header.stamp = ros::Time::now();
    //contact_msg.header.frame_id = "/cheetah/contact";
    std::vector<inekf_msgs::Contact> contacts;
    for (int i = 0; i < 4; i++)
    {
      inekf_msgs::Contact ct;
      ct.id = i;
      ct.indicator =  msg->f_foot[i][2] > sim_contact_force_threshold_ ;
      contacts.push_back(ct);
    }
    contact_msg.contacts = contacts;

    geometry_msgs::PoseStamped sim_pose_msg;
    sim_pose_msg.header.seq = seq_sim_;
    sim_pose_msg.header.stamp = ros::Time::now();
    sim_pose_msg.header.frame_id = "/cheetah/imu";
    sim_pose_msg.pose.position.x = msg->p[0];
    sim_pose_msg.pose.position.y = msg->p[1];
    sim_pose_msg.pose.position.z = msg->p[2];
    sim_pose_msg.pose.orientation.w = msg->quat[0];
    sim_pose_msg.pose.orientation.x = msg->quat[1];
    sim_pose_msg.pose.orientation.y = msg->quat[2];
    sim_pose_msg.pose.orientation.z = msg->quat[3];

    
    sim_contact_publisher_.publish(contact_msg);
    sim_pose_publisher_.publish(sim_pose_msg);
    
    
  }



} // mini_cheetah

template class cheetah_inekf_ros::KinematicsPublisher<12>;
template class cheetah_inekf_ros::InEKF_lcm<12>;
