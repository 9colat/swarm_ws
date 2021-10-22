#! /bin/bash
cd
cd swarm_ws/
git pull origin master

date > ~/check.txt
du -sh /home/ >> ~/check.txt
