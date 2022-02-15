#! /bin/bash

ip=$1

#nr=1
echo "logger"
roslaunch swarm_robot_nav on_robot_w_PS4.launch ## should be replaced with the final launch command

echo "sending"
cd
cd test_data
let nr=$(ls -l | grep -v ^d | wc -l)-2

scp ~/test_data/log$nr.txt ubuntu@192.168.0.$ip:/home/ubuntu/data
