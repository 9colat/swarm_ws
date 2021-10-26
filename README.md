# Swarm_ws
when you want to run the code with the teensy/arduino then you have to run a set of comands in the terminal.
frist you have to start the ros master:

roscore

then you have to set up the conection with the serial port:

rosrun rosserial_python serial_node.py /dev/ttyACM0

note that the port might differ between mechines so to find it you go to the arduino ide and go to the tool/port and there you will find the port your teensy/arduino is conected to.  

to run the test controller code then then run the following comands

rosrun hardware_driver driver.py

>if this does not work then go to the swarm_ws/src/hardware_driver/scripts and run the following comande, the reason that this is needed is because the system does not concider the script executable:

chmod 777 driver.py

ssh ubuntu@ip

add small fix what ever more more
hej
jk
why are you gay?
test
