#! /bin/bash

cd
cd swarm_ws/
git pull origin master
catkin_make
cd

source ~/swarm_ws/devel/setup

./swarm_ws/bash/teensy_push.sh $1

echo "thanks for the waiting, i have now finished"
