#! /bin/bash

cd
cd swarm_ws/arduino_code/lib_test/ros_lib
perl -pi -e 's/nicoleg/ubuntu/g' *.h
perl -pi -e 's/nicoleg/ubuntu/g' *.cpp
cd actionlib
perl -pi -e 's/nicoleg/ubuntu/g' *.h
cd ..
cd actionlib_msgs
perl -pi -e 's/nicoleg/ubuntu/g' *.h
