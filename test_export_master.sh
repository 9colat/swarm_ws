#! /bin/bash
echo "hey"
gnome-terminal -x sh -c "./launch.sh; bash"
sleep 1
export ROS_MASTER_URI=http://hal:11311
roslaunch hardware_driver test_1.launch
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
