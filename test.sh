#! /bin/bash

echo "å"|sudo -S /path/to/command

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
    sudo apt update -y
    sudo apt upgrade -y
    sudo apt install ros-noetic-desktop -y
    source /opt/ros/noetic/setup.bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
    sudo apt install python3-rosdep -y
    sudo rosdep init
    rosdep update
fi

sudo apt install ros-noetic-rosserial-arduino
sudo apt install ros-noetic-rosserial


echo "Download and installing arduino ide"
cd Download
curl -o arduino-1.8.16-linuxarm.tar.xz https://downloads.arduino.cc/arduino-1.8.16-linuxarm.tar.xz
tar -xf arduino-1.8.16-linuxarm.tar.xz -C /home/$USER/
cd
cd arduino-1.8.16
./install.sh
./arduino-linux-setup.sh

cd
cd Arduino/libraries/
rosrun rosserial_arduino make_libraries.py .

echo "installing Teensy"
curl -o 00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
curl -o TeensyduinoInstall.linuxarm https://www.pjrc.com/teensy/td_155/TeensyduinoInstall.linuxarm
chmod 755 TeensyduinoInstall.linuxarm
./TeensyduinoInstall.linuxarm --dir=arduino-1.8.16
cd arduino-1.8.16/hardware/teensy/avr/cores/teensy4
make
