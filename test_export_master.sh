#! /bin/bash
echo "hey"
gnome-terminal -x sh -c "./launch.sh; bash"
sleep 1
export ROS_MASTER_URI=http://hal:11311
rosrun hardware_driver driver.py
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
