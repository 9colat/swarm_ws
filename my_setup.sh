#! /bin/bash

#here we setup the git reposetory that we have made.
cd
git clone https://github.com/9colat/swarm_ws.git
cd swarm_ws
git checkout Pi_v1
catkin_make
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc

#here we setup the lidar
ls -l /dev | grep ttyUSB
sudo chmod 666 /dev/ttyUSB0
cd
cd swarm_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ..
catkin_make
source devel/setup.bash
