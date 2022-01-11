# Swarm_ws
The code on this repository is a part of a project done by:
-Aleksandra Dudlik
-Casper L. F. Mikkelsen
-Nicolaj T. Sørensen


##Install instruction
first fix your repository settings so that ros can be installed:
```
sudo nano /etc/apt/sources.list
```
go to the bottom and uncomment the 3 last lines that are commented out. (# means that its commented out)

next you need to get the "my_setup.sh" script this should set everything up for you, the same way that we have it set up. If you are on the full server version without a gui, then i recument that you make a folder and change directory to that so that you can keep the "home" folder a little clean:
```
curl -o my_setup.sh https://raw.githubusercontent.com/9colat/swarm_ws/master/my_setup.sh
```
now make it executable:
```
chmod +x my_setup.sh
```
now that should install everything that we used to make our robot run.

##User instruction:

when you want to run the code with the teensy/arduino then you have to run a set of comands in the terminal.
first you have to start the ros master:
```
roscore
```
then you have to set up the conection with the serial port:
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
note that the port might differ between mechines so to find it you go to the arduino ide and go to the tool/port and there you will find the port your teensy/arduino is conected to.  

to run the test controller code then then run the following comands
```
rosrun hardware_driver driver.py
```
>if this does not work then go to the swarm_ws/src/hardware_driver/scripts and run the following comande, the reason that this is needed is because the system does not concider the script executable:
```
chmod +x driver.py
```
ssh ubuntu@ip



for tele op:
```
roslaunch ds4_driver ds4_twist.launch
```
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
```
rostopic echo /cmd_vel
```
