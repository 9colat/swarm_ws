# Swarm_ws
when you want to run the code with the teensy/arduino then you have to run a set of comands in the terminal.
first you have to start the ros master:

roscore

then you have to set up the conection with the serial port:

rosrun rosserial_python serial_node.py /dev/ttyACM0

note that the port might differ between mechines so to find it you go to the arduino ide and go to the tool/port and there you will find the port your teensy/arduino is conected to.  

to run the test controller code then then run the following comands

rosrun hardware_driver driver.py

>if this does not work then go to the swarm_ws/src/hardware_driver/scripts and run the following comande, the reason that this is needed is because the system does not concider the script executable:

chmod 777 driver.py

ssh ubuntu@ip
ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQCk0mw8PVa00e9XQmEksm77+NieO8pwDGXSheLAyK8PkcDDsTv3OodFuxx0ca0Tx79rq7j05Q7lEKmXtZ2UiwkP7H3q8v/YXoOQgQyiVqJjUrTfN4JwLQbPoVkZTMuyGP6+iTlCr9VCNv0W9nE3t2/1GXUKZNUKGP/vEqggAFCBQ0+GARh0OosWP3NBeJztSzJqtQpb6B0g7OFbqEQLiJJ/PMxn5vUVUpmsi0vVbNch6VzC4n6dURH5cXvIzU2lm0cljObncxH7EYRtXfHWxRyICMOmdD7hO6OWzqvrEbGOEBLsJjvD7ZNtLvWSqQaOaPrsb7rvQM5BeEAJ6h7/VxcBzaaI68ljLULG6IHRNQlW2+4t4Amg6DxYy1zlsnw/ZcsGBBt4hUsNthnvnkn/AmCLIHSoxQyn6y7sCzCS75NBChyaygnq7HTNiCLngL2qbyvp9PpUglaIBLW4iK1OEN8R2BgpCjKfGKNol2LaQp4sE43YIPI81SFyz2D7iiGdxLc= nicoleg@nicoleg-ThinkPad-X250
