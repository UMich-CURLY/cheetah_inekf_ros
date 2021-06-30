# cheetah_inekf_ros
This repository contains a ROS package that subscribe to LCM messages, compute forward kinmatics for an MIT mini cheetah, and publish the kinematics, imu, and cotnact information to ROS.\
We provide a launch file `cheetah_lcm_inekf.launch` to run the invariant EKF for mini cheetah. (With LCM messages as input.)

## Dependencies
* ROS
* [invariant-ekf/devel](https://github.com/UMich-CURLY/invariant-ekf/tree/devel)
* [invariant-ekf-ros/minicheetah](https://github.com/UMich-CURLY/invariant-ekf-ros/tree/minicheetah)
* Eigen3
* [lcm 1.4.0](https://github.com/lcm-proj/lcm/releases/tag/v1.4.0)
* Boost

## Setup
### invaraint-ekf
* [invariant-ekf](https://github.com/UMich-CURLY/invariant-ekf/tree/devel) contains the core algorithm in C++. It is not a ROS package. You'll have to compile it as a standard cmake project.
```
git clone https://github.com/UMich-CURLY/invariant-ekf/
git checkout devel
mkdir build
cd build
cmake ..
make 
```
### invariant-ekf-ros
* [invariant-ekf-ros](https://github.com/UMich-CURLY/invariant-ekf-ros/tree/minicheetah) is a ROS wrapper for [invariant-ekf](https://github.com/UMich-CURLY/invariant-ekf/tree/devel). It subscribes to kinematics, imu, and contacts from ROS and parses it to [invariant-ekf](https://github.com/UMich-CURLY/invariant-ekf/tree/devel).\
* Clone it under your catkin source directory and switch the branch `minicheetah`.
```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone https://github.com/UMich-CURLY/invariant-ekf-ros
git checkout minicheetah
```
### cheetah_inekf_ros
* [cheetah_inekf_ros](https://github.com/UMich-CURLY/cheetah_inekf_ros) (this repository) subscribe sensor messages from LCM and publish kinematics, imu, and contacts in ROS for [invariant-ekf-ros](https://github.com/UMich-CURLY/invariant-ekf-ros/tree/minicheetah).\
* Please also clone it under your catkin source directory.
```
cd catkin_ws/src
git clone https://github.com/UMich-CURLY/cheetah_inekf_ros
```
* To build [invariant-ekf-ros](https://github.com/UMich-CURLY/invariant-ekf-ros/tree/minicheetah) and [cheetah_inekf_ros](https://github.com/UMich-CURLY/cheetah_inekf_ros), you'll have to do catkin_make twice. The program will compile user-defined ROS messages for the first time, which are the dependencies of the package.
```
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False
```

## Run invariant EKF with LCM
* `launch/cheetah_lcm_inekf.launch` is a launch file to start the invariant EKF with LCM messages. This launch file calls invariant-ekf-ros and run the filter automatically. The trajectory will be saved in both [kitti](https://github.com/MichaelGrupp/evo/wiki/Formats#kitti---kitti-dataset-pose-format) and [tum](https://github.com/MichaelGrupp/evo/wiki/Formats#tum---tum-rgb-d-dataset-trajectory-format) format. The save path can be modified in [line 32](https://github.com/UMich-CURLY/cheetah_inekf_ros/blob/main/launch/cheetah_lcm_inekf.launch#L32) and [line 33](https://github.com/UMich-CURLY/cheetah_inekf_ros/blob/main/launch/cheetah_lcm_inekf.launch#L33) of the lunch file.

* To run the code, run:
```
source catkin_ws/devel/setup.bash
roslaunch cheetah_inekf_ros cheetah_lcm_inekf.launch
```
* To play the LCM log: 
```
lcm-logplayer-gui data/08292020_trial1.log
```
