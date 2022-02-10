#! /bin/bash

nr=1
#echo "log$nr.txt"
echo "logger"
roslaunch swarm_robot_nav on_robot_w_PS4.launch ## should be replaced with the final launch command

echo "sending"
cd
cd test_data
#let nr=$(ls -l | grep -v ^d | wc -l)-2




scp ~/test_data/log$nr.txt ubuntu@192.168.0.100:/home/ubuntu/data
