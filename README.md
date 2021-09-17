# swarm_ws
when you want to run the code with the teensy/arduino then you have to run a set of camands in the terminal.
frist you have to start the ros master:

roscore

then you have to set up the conection with the serial port:

rosrun rosserial_python serial_node.py /dev/ttyACM0

note that the port might differ between mechines so to find it you go to the arduino ide and go to the tool/port and there you will find the port your teensy/arduino is conected to.  
