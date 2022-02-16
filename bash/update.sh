#! /bin/bash

if [$# -eq 0]
  then
    input_1=1
    name=motor_strait_driving
fi
if [$1 -gt 0]
  then
    input_1=$1
    if [$1 -eq 1]
      then
        name=motor_strait_driving
    fi
    if [$1 -eq 2]
      then
        name=motor_control
fi


cd
cd swarm_ws/
git pull origin master
catkin_make
cd

source ~/swarm_ws/devel/setup

./swarm_ws/bash/teensy_push.sh $input_1

echo "thanks for the waiting, i have now finished"
