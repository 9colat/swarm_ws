#! /bin/bash
echo "hey"
echo "export PATH=\$PATH:/home/pi/bin" >> ~/.bashrc
gnome-terminal -x sh -c "./test2.sh; bash"
echo "whats wrong with your face?"
