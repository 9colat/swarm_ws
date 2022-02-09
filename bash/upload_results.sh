#! /bin/bash

#nr=$1
#echo "log$nr.txt"
echo "logger"
rosrun hardware_driver logger_node.py

echo "sending"
cd
cd test_data
let nr=$(ls -l | grep -v ^d | wc -l)-2


scp ~/test_data/log$nr.txt nicoleg@192.168.0.9:/home/nicoleg/Documents
