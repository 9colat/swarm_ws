#! /bin/bash

cd

arduino-cli compile --fqbn teensy:avr:teensy36 ~/swarm_ws/arduino_code/${1}/${1}.ino
addad="no"
if grep -q "!arduino_code/${1}/build" ~/swarm_ws/.gitignore
  then
    addad="yes"
fi
if [ $addad == "no" ]
  then
      echo "\!arduino_code/${1}/build" >> ~/swarm_ws/.gitignore
fi

cd
cd swarm_ws
#git commit -m "auto upload from arduino maker"
# git push origin master
