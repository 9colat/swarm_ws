#! /bin/bash
IP=$1
echo "im starting to log the data"
#gnome-terminal -x sh -c "./launch.sh; bash"
read A B C D <<<"${IP//./ }"
export ROS_IP=$A.$B.$C.$D
export ROS_MASTER_URI=http://$A.$B.$C.$D:11311
rqt_graph
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
