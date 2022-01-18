#!/usr/bin/env python3
import rospy
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion


path = "~/test_data/log%s.txt"
number_of_files = 0
lidar_array = [0] * 360
wheel_speed = [0] * 2
lidar_label = ",lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar,lidar"
seperator = ","

def file_iterator():
    global number_of_files
    number_of_files = 0
    print("im finding out how many logs there are")
    while os.path.exists(path % number_of_files):
        print("im running itorator")
        number_of_files = number_of_files + 1

def callback(data):
    print(data.data)

def callback_lidar(data):
    global lidar_array
    print("laser data recived")
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]

def callback_wheel_speed(data):
    global wheel_speed
    wheel_speed[0] = data.x
    wheel_speed[1] = data.y



def main():
    global path, number_of_files, seperator
    file_iterator()
    rospy.init_node('logger', anonymous=True)
    print(str(lidar_array))
    f = open(path % number_of_files,"a")
    f.write("Right wheel speed"+seperator+"Left wheel speed"+lidar_label+"\n")
    f.close()
    #rospy.Subscriber("chatter", String, callback)
    v = 1
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(v)
        rospy.Subscriber("scan", LaserScan, callback_lidar)
        rospy.Subscriber("speed_and_tick", Quaternion, callback_wheel_speed)

        f = open(path % number_of_files,"a")
        for i in range(len(wheel_speed)):
            f.write(str(wheel_speed[i])+seperator)
        for j in range(len(lidar_array)-1):
            f.write(str(lidar_array[j])+seperator)
        f.write(str(lidar_array[359])+"\n")
        f.close()
        v += 1
        rate.sleep()
    #rospy.spin()

if __name__ == '__main__':
    main()