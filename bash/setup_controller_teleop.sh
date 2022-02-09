#! /bin/bash

sudo apt-get install pi-bluetooth
sudo apt-get install bluetooth bluez blueman
# A reboot is needed after this commands to make sure that the bluetooth works
# on the pi but the rest of the scripst can still run to install every thing.
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
cd ds4drv
mkdir -p ~/.local/lib/python3.8/site-packages
python3 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo pip install ds4drv

cd

cd swarm_ws

catkin_make


sudo reboot
