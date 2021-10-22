#! /bin/bash
echo "å"|sudo -S /path/to/command
sudo apt update -y
sudo apt upgrade -y

cd
cd swarm_ws/
git pull origin master

date > ~/check.txt
du -sh /home/ >> ~/check.txt
