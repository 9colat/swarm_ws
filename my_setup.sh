#! /bin/bash

################################################################################
################################################################################
################################################################################
################################################################################
################################################################################
# If you have not added the Repositories for mulitivers, universe and main
# restricted this will not work.
################################################################################
################################################################################
################################################################################
################################################################################
################################################################################

echo "ros2021"|sudo -S /path/to/command

echo "Setting up wifi for server version of the os"
sudo apt install network-manager -y

ncmli r wifi on

sudo nmcli dev wifi connect testwifi

echo "here ros is installing"
#sudo add-apt-repository "deb-src http://security.ubuntu.com/ubuntu focal-security main restricted"
#sudo add-apt-repository "deb-src http://security.ubuntu.com/ubuntu focal-security universe"
#sudo add-apt-repository "deb-src http://security.ubuntu.com/ubuntu focal-security multiverse"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
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
source ~/.bashrc

echo "setting up the arduino ide and libraries"
sudo apt install ros-noetic-rosserial-arduino -y
sudo apt install ros-noetic-rosserial -y
sudo apt install ros-noetic-move-base -y
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt install ros-noetic-navigation -y

sudo apt update -y
sudo apt upgrade -y

echo "Download and installing arduino ide"
cd
cd Downloads/
curl -o arduino-1.8.15-linuxaarch64.tar.xz https://downloads.arduino.cc/arduino-1.8.15-linuxaarch64.tar.xz
tar -xf arduino-1.8.15-linuxaarch64.tar.xz -C /home/$USER/
cd
cd arduino-1.8.15
sudo ./install.sh
./arduino-linux-setup.sh $USER

cd
cd arduino-1.8.15/libraries/
rosrun rosserial_arduino make_libraries.py .
cd


echo "installing Teensy"
cd
curl -o 00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
curl -o TeensyduinoInstall.linuxaarch64 https://www.pjrc.com/teensy/td_155/TeensyduinoInstall.linuxaarch64
chmod 755 TeensyduinoInstall.linuxaarch64
./TeensyduinoInstall.linuxaarch64 --dir=arduino-1.8.15
cd arduino-1.8.15/hardware/teensy/avr/cores/teensy3
perl -pi -e 's/MCU=MK20DX256/MCU=MK66FX1M0/g' Makefile
make

#here we setup the git reposetory that we have made.
cd
git clone https://github.com/9colat/swarm_ws.git
cd swarm_ws
#git checkout Pi_v1
cd src
catkin_init_workspace
cd ..
catkin_make
echo "source $HOME/swarm_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

#here we setup the lidar
ls -l /dev | grep ttyUSB
sudo chmod 666 /dev/ttyUSB0
cd
cd swarm_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ..
catkin_make
source devel/setup.bash
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
catkin_make
source devel/setup.bash
cd



#installing snap package mangerger and the arduino cli
sudo apt install snapd
sudo snap install arduino-cli

sudo apt install teensy-loader-cli -y
cd
cd swarm_ws
sudo chmod 777 arduino_make_upload.sh
sudo chmod 777 start_up_script.sh

cd

sudo apt-get install qt5-default -y

sudo apt install python3-pip

pip install pathlib

cronjob="@reboot ~/swarm_ws/start_up_script.sh"
(crontab -u $USER -l; echo "$cronjob" ) | crontab -u $USER -
