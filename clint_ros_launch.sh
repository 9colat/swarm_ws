#! /bin/bash
IP=$1
echo "hey"
#gnome-terminal -x sh -c "./launch.sh; bash"
read A B C D <<<"${IP//./ }"
export ROS_IP=$A.$B.$C.$D
export ROS_MASTER_URI=http://$A.$B.$C.$D:11311
roslaunch ds4_driver ds4_twist.launch
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
