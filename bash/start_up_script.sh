#! /bin/bash
echo "Im starting"
sleep 30

if [ $# -eq 0 ]
  then
    name=motor_strait_driving
fi
if [ $# -gt 0 ]
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

#echo "å"|sudo -S /path/to/command
#a visable command
echo "Here we go"
sudo apt update -y
sudo apt upgrade -y
echo "doing the git stuff"
cd
cd swarm_ws/
git pull origin master
catkin_make
cd

teensy_loader_cli --mcu=mk66fx1m0 -s /home/$USER/swarm_ws/arduino_code/${name}/build/teensy.avr.teensy36/${name}.ino.hex

source ~/swarm_ws/devel/setup.bash
#cd swarm_ws
#gnome-terminal -x sh -c "./test1.sh; bash"
#roslaunch hardware_driver test.launch
tm=$(date)
echo "${tm} hey im working" >> time_stamp.txt




#date > ~/check.txt
#du -sh /home/ >> ~/check.txt
