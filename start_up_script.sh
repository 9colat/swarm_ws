#! /bin/bash
sleep 30

#echo "å"|sudo -S /path/to/command
sudo apt update -y
sudo apt upgrade -y

cd
cd swarm_ws/
git pull origin master
catkin_make
cd

source ~/swarm_ws/devel/setup.bash
#cd swarm_ws
#gnome-terminal -x sh -c "./test1.sh; bash"
#roslaunch hardware_driver test.launch




#date > ~/check.txt
#du -sh /home/ >> ~/check.txt
