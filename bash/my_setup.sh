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

#echo "ros2021"|sudo -S /path/to/command
echo "updating and upgrading the system"
sudo apt update -y
sudo apt upgrade -y

echo "Setting up wifi for server version of the os"
sudo apt install network-manager -y

nmcli r wifi on

sudo nmcli dev wifi connect testwifi1 password "ros123456789"

echo "here ros is installing"
echo 'deb-src http://ports.ubuntu.com/ubuntu-ports focal-security main restricted' | sudo tee --append /etc/apt/sources.list
echo 'deb-src http://ports.ubuntu.com/ubuntu-ports focal-security universe'| sudo tee --append /etc/apt/sources.list
echo 'deb-src http://ports.ubuntu.com/ubuntu-ports focal-security multiverse'| sudo tee --append /etc/apt/sources.list
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
rm 00-teensy.rules
curl -o TeensyduinoInstall.linuxaarch64 https://www.pjrc.com/teensy/td_155/TeensyduinoInstall.linuxaarch64
chmod 755 TeensyduinoInstall.linuxaarch64
./TeensyduinoInstall.linuxaarch64 --dir=arduino-1.8.15
cd arduino-1.8.15/hardware/teensy/avr/cores/teensy3
perl -pi -e 's/MCU=MK20DX256/MCU=MK66FX1M0/g' Makefile
make

cd
echo "installing qt5"
sudo apt-get install qt5-default -y
echo "installing pip"
sudo apt install python3-pip -y
echo "pip installing pathlib"
pip install pathlib
echo "pip install ds4drv"
pip install ds4drv
curl -o 50-ds4drv.rules https://raw.githubusercontent.com/naoki-mizuno/ds4drv/devel/udev/50-ds4drv.rules
sudo cp 50-ds4drv.rules /etc/udev/rules.d/
rm 50-ds4drv.rules

#here we setup the git reposetory that we have made.
echo "clonning the github repo"
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
#cd
#cd swarm_ws/src
#git clone https://github.com/Slamtec/rplidar_ros.git
#catkin_make

git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
cd ..
catkin_make
source devel/setup.bash
cd



#installing snap package mangerger and the arduino cli
sudo apt install snapd
sudo snap install arduino-cli

sudo apt install teensy-loader-cli -y
cd
cd swarm_ws
##sudo chmod 777 arduino_make_upload.sh
##sudo chmod 777 start_up_script.sh

##sudo snap install hub-ctrl


echo 'include btcfg.txt' | sudo tee --append /boot/firmware/usercfg.txt

#btattach -B /dev/ttyAMA0 -P bcm -S 115200 -N &


cronjob="@reboot ~/swarm_ws/start_up_script.sh"
(crontab -u $USER -l; echo "$cronjob" ) | crontab -u $USER -
cronjob="@reboot ~/swarm_ws/bt_boot.sh"
(crontab -u $USER -l; echo "$cronjob" ) | crontab -u $USER -
