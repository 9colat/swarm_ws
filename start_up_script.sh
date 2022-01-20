#! /bin/bash
echo "Im starting"
sleep 30

#echo "å"|sudo -S /path/to/command
echo "Here we go"
sudo apt update -y
sudo apt upgrade -y
echo "doing the git stuff"
cd
cd swarm_ws/
git pull origin master
catkin_make
cd

source ~/swarm_ws/devel/setup.bash
#cd swarm_ws
#gnome-terminal -x sh -c "./test1.sh; bash"
#roslaunch hardware_driver test.launch
tm=$(date)
echo "${tm} hey im working" >> time_stamp.txt




#date > ~/check.txt
#du -sh /home/ >> ~/check.txt
