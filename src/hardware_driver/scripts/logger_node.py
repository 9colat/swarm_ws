#!/usr/bin/env python3
import rospy
import sys
import os
from pathlib import Path
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist




##### this is a subscriber example that can be copy pasta later #####

## paste this as a global variable, outside of the callback function (after "number_of_files = 0")
#data_to_be_logged = [0] * 3 # remember to specify this correctly, depending on what you are trying to log

## now we specify the callback function for the specific data subscription (replace "something" with what you want)
#def callback_something(data):

#    global data_to_be_logged
#    data_to_be_logged[0] = data.x
#    data_to_be_logged[1] = data.y
#    data_to_be_logged[2] = data.z

## past the thing  below in the main loop within "while not rospy.is_shutdown():"
#    rospy.Subscriber("topic_name", Data_Type, callback_something) #(topic name, data type, callback function)

##### now you should have a working subscriber! #####



input_speed = 0


path = Path.home().joinpath("test_data", "log%s.txt")
folder_path = str(Path.home().joinpath("test_data"))
isfolder = os.path.isdir(folder_path)
number_of_files = 0
lidar_array = [0] * 360
wheel_speed = [0] * 4
lidar_label = ",lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar"
seperator = ","

def file_iterator():
    global number_of_files
    number_of_files = 0
    #print("im finding out how many logs there are")
    while os.path.exists(str(path) % number_of_files):
        #print("im running itorator")
        number_of_files = number_of_files + 1

def callback(data):
    print(data.data)

def callback_lidar(data):#####
    global lidar_array
    #print("laser data recived")
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]

def callback_wheel_speed(data):
    global wheel_speed
    wheel_speed[0] = data.x
    wheel_speed[1] = data.y
    wheel_speed[2] = data.z
    wheel_speed[3] = data.w


def callback_input_speed(data):
    global input_speed
    input_speed = data.linear.x


def main():
    global path, folder_path, isfolder, number_of_files, seperator
    print(isfolder)
    if not isfolder:
        os.mkdir(folder_path)
        print("making directory")
    file_iterator()
    array_length = len(wheel_speed)
    rospy.init_node('logger', anonymous=True)
    #print(str(lidar_array))
    f = open(str(path) % number_of_files,"a")
    f.write("Right wheel speed"+seperator+"Left wheel speed"+seperator+"Input Omega Right"+seperator+"Input Linear Speed"+"\n")
    #f.write("Right wheel speed"+seperator+"Left wheel speed"+lidar_label+"\n")
    f.close()
    #rospy.Subscriber("chatter", String, callback)
    #v = 1
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        #print(v)
        #rospy.Subscriber("scan", LaserScan, callback_lidar)
        rospy.Subscriber("speed_and_tick", Quaternion, callback_wheel_speed)
        rospy.Subscriber("cmd_vel", Twist, callback_input_speed)

        f = open(str(path) % number_of_files,"a")
        #f.write(str(input_speed)+seperator)
        for i in range(array_length-1):
            f.write(str(wheel_speed[i])+seperator)
        f.write(str(wheel_speed[array_length-1])+"\n")
        #for j in range(len(lidar_array)-1):
        #    f.write(str(lidar_array[j])+seperator)
        #f.write(str(lidar_array[359])+"\n")
        f.close()
        #v += 1
        rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    main()
