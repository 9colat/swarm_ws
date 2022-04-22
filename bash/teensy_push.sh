#! /bin/bash

if [$# -eq 0]
  then
    name=motor_strait_driving
fi
if [$# -gt 0]
  then
    if [$1 -eq 1]
      then
        name=motor_strait_driving
    fi
    if [$1 -eq 2]
      then
        name=motor_control
    fi
fi
echo $name
teensy_loader_cli --mcu=mk66fx1m0 -s /home/$USER/swarm_ws/arduino_code/${name}/build/teensy.avr.teensy36/${name}.ino.hex
