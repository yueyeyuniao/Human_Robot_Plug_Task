# A Shared Control Method for Collaborative Human-Robot Plug Task (RAL+IROS 2021) [accepted]
## [Paper link](https://ieeexplore.ieee.org/document/9492826)
## [Video link](https://drive.google.com/file/d/1WQNfCKpU3OxEY8nxI3BN-lStOijAlSoF/view?usp=sharing)

## System overview:
![alt-text](https://github.com/yueyeyuniao/Human_Robot_Plug_Task/blob/main/media/intro_HRI.PNG)<br/>


## System requirement:
#### ROS Kinectic
#### Opencv
#### PCL
#### [Darknet_ros](https://github.com/leggedrobotics/darknet_ros)
#### [iai_kinect2](https://github.com/code-iai/iai_kinect2)
#### [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
#### [Kinova ros_kortex package](https://github.com/Kinovarobotics/ros_kortex)

## Hardware requirement:
#### Kinova gen3 arm
#### Realsense
#### Kinect
#### AprilTag
#### Socket
#### Cable with a plug

## Introduction:
#### /apriltags_ros folder includes how we use the apriltag to detect the socket
#### /hri_plug_task folder includes all the launch file, source codes for the plug task with human-robot interaction
#### /kalman_filter folder includes how we implement the Kalman filter to track the moving socket

## Commands
#### --launch robot and cameras
##### roslaunch hri_plug_task HRI_plug_task_gen3.launch
#### --launch apriltag
##### roslaunch apriltags_ros HRI_plug_task_gen3.launch
#### --run kalman filter
##### rosrun kalman_filter kalman_node
#### --cable modeling
##### rosrun hri_plug_task HRI_modeling_cable_gen3_kf
#### --task automation with human-in-the-loop
##### rosrun hri_plug_task HRI_plug_task_gen3_kf __ns:=my_gen3


## Copyright: 
### This work was developed at the [RIVeR Lab, Northeastern University](http://robot.neu.edu/) 



