#include "ros/ros.h"
#include <string>
#include <sstream> 
#include <fstream>
#include <array>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>

class PosePublisherNode{
    public:
        PosePublisherNode(ros::NodeHandle n) : n_(n) {
            // Create private node handle
            ros::NodeHandle nh("~");
            std::string pose_csv_file, init_rot_file;
            std::string pose_topic, pose_frame;

            nh.param<std::string>("pose_topic", pose_topic, "/cheetah/groundtruth/pose");
            nh.param<std::string>("pose_frame", pose_frame, "/odom");
            nh.param<std::string>("pose_csv_file", pose_csv_file, "/data/ground_truth.csv");
            nh.param<std::string>("init_rot_file", init_rot_file, "/data/init_rot_file.csv");
            nh.param<double>("publish_rate", publish_rate_, 1); 
            nh.param<int>("pose_skip", pose_skip_, 1); 
            
            // load poses from csv files

            std::cout<<"reading poses from csv"<<std::endl;
            pose_from_csv_ = readPoseFromCSV(pose_csv_file,init_rot_file);
            first_pose_ = pose_from_csv_.front();
            std::cout<<"done reading, it contains: "<<pose_from_csv_.size()<<" of poses."<<std::endl;
            std::cout<<"first pose is: "<<first_pose_[0]<<", "<<first_pose_[1]<<", "<<first_pose_[2]<<std::endl;
            pose_frame_ = pose_frame;
            
            pose_pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
            this->pose_publishing_thread_ = std::thread([this]{this->posePublishingThread();});
        }

    private:
        ros::NodeHandle n_;
        ros::Publisher pose_pub_;
        std::string pose_frame_;
        uint32_t seq_ = 0;
        double publish_rate_;
        int pose_skip_;
        std::queue<std::array<float,3>> pose_from_csv_;
        std::array<float,3> first_pose_;
        std::vector<geometry_msgs::PoseStamped> poses_;
        std::mutex poses_mutex_;
        std::thread pose_publishing_thread_;

        // Pose message callback
        void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
            if ((int)msg->header.seq%pose_skip_!=0) { return; }
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose = msg->pose.pose;
            std::lock_guard<std::mutex> lock(poses_mutex_);
            poses_.push_back(pose);
        }

        std::queue<std::array<float,3>> readPoseFromCSV(std::string pose_csv_file,std::string init_rot_file){
            Eigen::Matrix3f init_rot;
            std::string temp_rot;
            std::ifstream init_file(init_rot_file);
            if(init_file.is_open()){
                for(int i=0; i<3; ++i){
                    for(int j=0; j<3; ++j){
                        std::getline(init_file,temp_rot,',');
                        init_rot(i,j) = std::stof(temp_rot);
                    }
                }
            }
            init_file.close();

            std::cout<<"loaded init tf: "<<init_rot<<std::endl;
            std::ifstream csv_file(pose_csv_file);
            std::queue<std::array<float,3>> pose_output;

            if(csv_file.is_open()){
                std::string line;

                while(std::getline(csv_file, line))
                {
                    std::stringstream ss(line);
                    Eigen::Vector3f temp_pose;
                    std::array<float,3> temp_pose_after_tf;
                    int i = 0;
                    std::string temp_str;
                    while(std::getline(ss, temp_str, ',')){
                        temp_pose[i] = std::stof(temp_str);
                        i++;
                    }
                    temp_pose = init_rot * temp_pose;
                    for(int j=0; j<3; j++){
                        temp_pose_after_tf[j] = temp_pose[j];
                    }
                    
                    pose_output.push(temp_pose_after_tf);
                }
                csv_file.close();
            }
            return pose_output;
        }

        // Publishes path
        void posePublish() {
            if(!pose_from_csv_.empty()){
                std::array<float,3> cur_pose = pose_from_csv_.front();
                pose_from_csv_.pop();

                geometry_msgs::PoseWithCovarianceStamped pose_msg;
                pose_msg.header.seq = seq_;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = pose_frame_;
                pose_msg.pose.pose.position.x = cur_pose[0] - first_pose_[0];
                pose_msg.pose.pose.position.y = cur_pose[1] - first_pose_[1];
                pose_msg.pose.pose.position.z = cur_pose[2] - first_pose_[2];
                pose_msg.pose.pose.orientation.w = 0;
                pose_msg.pose.pose.orientation.x = 0;
                pose_msg.pose.pose.orientation.y = 0;  
                pose_msg.pose.pose.orientation.z = 0;
                // std::cout<<"publishing: "<<pose_msg.pose.pose.position.x<<", "<<pose_msg.pose.pose.position.y<<", "<<pose_msg.pose.pose.position.z<<std::endl;
                pose_pub_.publish(pose_msg);
                seq_++;
            }
        }

        // Path publishing thread
        void posePublishingThread(){
            // Loop and publish data
            ros::Rate loop_rate(publish_rate_);
            while(ros::ok()){
                posePublish();
                loop_rate.sleep();
            }
        }
};

int main(int argc, char **argv) {
    // Initialize ROS node
    ros::init(argc, argv, "pose_publisher");  
    ros::NodeHandle n;
    PosePublisherNode pose_publisher_node(n);
    ros::spin();
    return 0;
}