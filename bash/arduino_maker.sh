#! /bin/bash

arduino-cli compile --fqbn teensy:avr:teensy36 ~/swarm_ws/arduino_code/${1}/${1}.ino
