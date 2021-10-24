#! /bin/bash

echo "å"|sudo -S /path/to/command #remember to change to your password
echo "updating the system"
sudo apt update -y
sudo apt upgrade -y
echo "installing ros if its not there"
FILE1=/opt/ros/noetic/env.sh
if [ -f "$FILE1" ]; then
    echo "ros exists on the pc - nice."
else
    echo "ros does not exist, well here we go again ;)."
    sudo add-apt-repository "deb-src http://security.ubuntu.com/ubuntu focal-security main restricted"
    sudo add-apt-repository "deb-src http://security.ubuntu.com/ubuntu focal-security universe"
    sudo add-apt-repository "deb-src http://security.ubuntu.com/ubuntu focal-security multiverse"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl -y# if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt install ros-noetic-desktop -y
    source /opt/ros/noetic/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    sudo apt install python3-rosdep -y
    sudo rosdep init
    rosdep update
fi
echo "installing rosserial_arduino if its not there"
FILE2=/opt/ros/noetic/include/rosserial_arduino/Test.h
if [ -f "$FILE2" ]; then
    echo "rosserial_arduino exists on the pc - nice."
else
    sudo apt install ros-noetic-rosserial-arduino -y
    sudo apt install ros-noetic-rosserial -y
fi

echo "Download and installing arduino ide"
FILE3=/home/$USER/arduino-1.8.16
FILE35=/home/$USER/arduino-1.8.15
if [ -f "$FILE3" ] || [ -f "$FILE35" ]; then
    echo "arduino ide exists on the pc - nice."
else
    cd Downloads/
    curl -o arduino-1.8.16-linux64.tar.xz https://downloads.arduino.cc/arduino-1.8.16-linux64.tar.xz
    tar -xf arduino-1.8.16-linux64.tar.xz-C /home/$USER/
    cd
    cd arduino-1.8.16
    sudo ./install.sh
    ./arduino-linux-setup.sh $USER
    cd
    cd Arduino/libraries/
rosrun rosserial_arduino make_libraries.py .

echo "installing Teensy"
FILE4=/etc/udev/rules.d/00-teensy.rules
if [ -f "$FILE4" ]; then
    echo "teensy exists on the pc - nice."
else
    curl -o 00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
    sudo cp 00-teensy.rules /etc/udev/rules.d/
    curl -o TeensyduinoInstall.linux64 https://www.pjrc.com/teensy/td_155/TeensyduinoInstall.linux64
    chmod 755 TeensyduinoInstall.linux64
    ./TeensyduinoInstall.linux64 --dir=arduino-1.8.16
    cd arduino-1.8.16/hardware/teensy/avr/cores/teensy4
    make
    cd
    cd arduino-1.8.16/hardware/teensy/avr/cores/teensy3
    make

fi

FILE5=/home/$USER/swarm_ws/src/hardware_driver/scripts/driver.py
if [ -f "$FILE5" ]; then
    echo "swarm_ws exists on the pc - nice."
else
    cd
    git clone https://github.com/9colat/swarm_ws.git
    cd swarm_ws
    git checkout Pi_v1
    catkin_make
    echo "source $HOME/swarm_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
fi


#here we setup the lidar
FILE6=/home/$USER/swarm_ws/src/rplidar_ros/src/node.cpp
if [ -f "$FILE6" ]; then
    echo "rplidar_ros exists on the pc - nice."
else
    ls -l /dev | grep ttyUSB
    sudo chmod 666 /dev/ttyUSB0
    cd
    cd swarm_ws/src
    git clone https://github.com/Slamtec/rplidar_ros.git
    cd ..
    catkin_make
    source devel/setup.bash
fi

cd
sudo apt install arduino-mk -y
