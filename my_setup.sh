#! /bin/bash

#here er install ros
echo "å"|sudo -S /path/to/command

FILE=/opt/ros/noetic/env.sh
if [ -f "$FILE" ]; then
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


#here we setup the git reposetory that we have made.
cd
git clone https://github.com/9colat/swarm_ws.git
cd swarm_ws
git checkout Pi_v1
catkin_make
echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc
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
