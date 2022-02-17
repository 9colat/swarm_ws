#! /bin/bash

echo "hey"

in=$1

IP=$(hostname -I)
read A B C D <<<"${IP//./ }"
export ROS_IP=$A.$B.$C.$D
export ROS_MASTER_URI=http://$A.$B.$C.$D:11311
cd
./swarm_ws/bash/code_runner $in

#roslaunch swarm_robot_nav on_robot.launch
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
