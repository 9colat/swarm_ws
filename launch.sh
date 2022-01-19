#! /bin/bash
echo "${tm} roscore started" >> time_stamp.txt
roscore
echo "${tm} roscore stopped" >> time_stamp.txt
#sleep 60
#gnome-terminal -x sh -c "./test1.sh; bash"
#echo "whats wrong with your face?"
#kill $PPID
