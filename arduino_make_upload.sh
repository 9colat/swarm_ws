#! /bin/bash
cd
echo "press the button on the teensy you have 10 sec"
sleep 5
echo "5"
sleep 1
echo "4"
sleep 1
echo "3"
sleep 1
echo "2"
sleep 1
echo "1"
sleep 1
echo "0 - i hope you pressed it"
arduino-cli compile --fqbn teensy:avr:teensy36 swarm_ws/arduino_code/motor_control/motor_control.ino
echo "done with compiling"
teensy_loader_cli --mcu=mk66fx1m0 -w /home/$USER/swarm_ws/arduino_code/motor_control/build/teensy.avr.teensy36/motor_control.ino.hex
