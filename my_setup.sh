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
    sudo apt install curl # if you haven't already installed curl
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

#setting up the arduino ide and libraries
sudo apt install ros-noetic-rosserial-arduino -y
sudo apt install ros-noetic-rosserial -y
sudo apt update -y
sudo apt upgrade -y

echo "Download and installing arduino ide"
cd Downloads/
curl -o arduino-1.8.16-linuxaarch64.tar.xz https://downloads.arduino.cc/arduino-1.8.16-linuxaarch64.tar.xz
tar -xf arduino-1.8.16-linuxaarch64.tar.xz -C /home/$USER/
cd
cd arduino-1.8.16
sudo ./install.sh
./arduino-linux-setup.sh $USER

cd
cd arduino-1.8.16/libraries/
rosrun rosserial_arduino make_libraries.py .

echo "installing Teensy"
curl -o 00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
curl -o TeensyduinoInstall.linuxaarch64 https://www.pjrc.com/teensy/td_155/TeensyduinoInstall.linuxaarch64
chmod 755 TeensyduinoInstall.linuxaarch64
./TeensyduinoInstall.linuxaarch64 --dir=arduino-1.8.16
cd arduino-1.8.16/hardware/teensy/avr/cores/teensy4
make

#here we setup the git reposetory that we have made.
cd
git clone https://github.com/9colat/swarm_ws.git
cd swarm_ws
git checkout Pi_v1
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

sudo cp ~/swarm_ws/disk-space-check.sh /usr/local/bin/disk-space-check.sh
sudo chmod 744 /usr/local/bin/disk-space-check.sh
sudo chmod 664 /etc/systemd/system/disk-space-check.service

#cd
#sudo apt install arduino-mk -y
#cd swarm_ws/arduino_code/motor_control
#make upload


#ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCk0mw8PVa00e9XQmEksm77+NieO8pwDGXSheLAyK8PkcDDsTv3OodFuxx0ca0Tx79rq7j05Q7lEKmXtZ2UiwkP7H3q8v/YXoOQgQyiVqJjUrTfN4JwLQbPoVkZTMuyGP6+iTlCr9VCNv0W9nE3t2/1GXUKZNUKGP/vEqggAFCBQ0+GARh0OosWP3NBeJztSzJqtQpb6B0g7OFbqEQLiJJ/PMxn5vUVUpmsi0vVbNch6VzC4n6dURH5cXvIzU2lm0cljObncxH7EYRtXfHWxRyICMOmdD7hO6OWzqvrEbGOEBLsJjvD7ZNtLvWSqQaOaPrsb7rvQM5BeEAJ6h7/VxcBzaaI68ljLULG6IHRNQlW2+4t4Amg6DxYy1zlsnw/ZcsGBBt4hUsNthnvnkn/AmCLIHSoxQyn6y7sCzCS75NBChyaygnq7HTNiCLngL2qbyvp9PpUglaIBLW4iK1OEN8R2BgpCjKfGKNol2LaQp4sE43YIPI81SFyz2D7iiGdxLc= nicoleg@nicoleg-ThinkPad-X250
