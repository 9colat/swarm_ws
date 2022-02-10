#! /bin/bash

nr=1
#echo "log$nr.txt"
echo "logger"
rosrun hardware_driver logger_node.py ## should be replaced with the final launch command

echo "sending"
cd
cd test_data
#let nr=$(ls -l | grep -v ^d | wc -l)-2



scp ~/test_data/log$nr.txt ubuntu@192.168.0.100:/home/ubuntu/data
