#! /bin/bash
echo "hey"
#gnome-terminal -x sh -c "./launch.sh; bash"
export ROS_MASTER_URI=http://192.168.0.5:11311
roslaunch hardware_driver test_1.launch
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
