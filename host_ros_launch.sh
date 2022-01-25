#! /bin/bash
echo "hey"

IP=$(hostname -I)
read A B C D <<<"${IP//./ }"
export ROS_IP=$A.$B.$C.$D
export ROS_MASTER_URI=http://$A.$B.$C.$D:11311
roslaunch hardware_driver test_for_driving.launch
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
