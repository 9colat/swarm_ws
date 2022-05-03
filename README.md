# Swarm_ws
The code on this repository is a part of a project done by:

-Aleksandra Dudlik

-Casper L. F. Mikkelsen

-Nicolaj T. Sørensen

## ssh setup

First, it is a good idea to install network-map on your PC, because you do not need a RP-screen, to get the IP address of you raspberry pi:
```
sudo apt install nmap
```
This allows you to search the network devices on WIFI/LAN access points.
In order to find the IP address, use this command to find all the host on the net work that is from 192.168.0.0 to 192.168.0.255 (remember to find out your router's IP first!):
```
nmap 192.168.0.*
```
If you don't have a ssh-key at this point, then you have to make one. This can be done with this command:
```
ssh-keygen
```
This allows the system to give the key, so we are seen now as a 'safe' device.
(this command you use only once, regardless of how many devices you want to ssh into)
This key needs to be saved in a specific file (just press ENTER three times). After this step, your ssh identifier has been successfully generated.

Now, we copy the generated key (ssh identifier) to the host, so it will not ask us for password in the future. Use this command, but remember to replace the remote_host with the IP address that you found earlier (e.g. 192.168.0.3):
```
ssh-copy-id ubuntu@remote_host
```

Now you can ssh into the machine with the following command (here you also replace the remote_host with the IP address that you found earlier, like in the previous step):
```
ssh ubuntu@remote_host
```

## Install instruction
> First, fix your repository settings, so ROS can be installed:
> ~~sudo nano /etc/apt/sources.list~~
> ~~go to the bottom and uncomment the 3 last lines that are commented out. (# means that it is commented out)~~
> This is not the case any more, and you can skip the first step of the install instruction.

The next step requires "my_setup.sh" script, which installs all neccessary software and packages needed for this project. The script should set everything up for you, the same way that we have set it up. If you are on the full server version without a GUI, then i recommend that you make a folder and change directory to that, so you can keep the "home" folder a little clean.
Download the script:
```
curl -o my_setup.sh https://raw.githubusercontent.com/9colat/swarm_ws/master/my_setup.sh
```
And make it executable:
```
chmod +x my_setup.sh
```
You can quickly test if it is executable by running "ls" command. If it appears in a green colour (in terminator), then it means it is indeed executable.

Now the only thing left is to install everything that we used to make our robot run, by using the "my_setup.sh" script:

```
./my_setup.sh
```

It will take some time, roughly around 90 minutes. Enjoy yourself.

## User instruction:

When you want to run the code with the Teensy/Arduino, then you have to run a set of commands in the terminal.

```
./swarm_ws/bash/code_runner.sh
```




--- out of data and debug stuff ---
First, you need to start the ROS master:
```
roscore
```
Then you have to set up the connection with the serial port:
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Note that the port might differ between machines. To find it, you need to go to the Arduino IDE and click on the tool/port - there you find the port your Teensy/Arduino is connected to.  

To run the test controller code, run the following commands:
```
rosrun hardware_driver driver.py
```
>if this does not work then go to the swarm_ws/src/hardware_driver/scripts and run the following command; the reason that this is needed is because the system does not consider the script executable:
```
chmod +x driver.py
```

## bluetooth setting up:

```
sudo bluetoothctl
```

```
bluetoothctl connect D0:27:88:70:E1:D3
```

```
pair D0:27:88:70:E1:D3
```

```
connect D0:27:88:70:E1:D3
```


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


## Static Test launch procedure
The test is preformed with having the robot platform sit at multiple locations and collected data that is then logged. The log data file will be analysis at the end.


First you will need to launch a roscore
```
roscore
```
After you will need to run this
```
rosrun hardware_driver pylaunch.py
```
## linear motion Test launch procedure
```
sudo nmcli dev wifi connect "Smart production" password "aau smart production lab"
```
sudo nmcli dev wifi connect "Piss off_my_hotspot" password "hello there"
