#! /bin/bash
echo "hey"
echo "$(date)" >> ~/time_stamp.txt
sleep 60
gnome-terminal -x sh -c "./test1.sh; bash"
echo "whats wrong with your face?"
kill $PPID
