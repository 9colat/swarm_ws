#! /bin/bash

sudo apt install ros-noetic-robot-upstart -y
source /opt/ros/noetic/setup.bash
## here we have to change the line for the launch file that we want to launch.
rosrun robot_upstart install hardware_driver/launch/test.launch
