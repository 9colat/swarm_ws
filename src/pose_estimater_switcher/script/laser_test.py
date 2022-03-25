#!/usr/bin/env python3
import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
from laser_part_of_the_system import Laser_component
from sensor_msgs.msg import LaserScan
lidar_array = [0.0] * 360

#print("dav")

def callback_lidar(data):
    global lidar_array
    print("lidar is running")
    for i in range(len(data.ranges)):
        lidar_array[i] = data.ranges[i]




def main():
    global lidar_array
    w2 = Laser_component()

    theta_r = math.radians(50)
    theta_m = math.radians(30)
    r_x = 15234
    r_y = 11140
    x_m = math.cos(theta_m)
    y_m = math.sin(theta_m)
    m_t = math.atan2(y_m, x_m)
    r_h_x = math.cos(theta_r + m_t)
    r_h_y = math.sin(theta_r + m_t)

    b_x = 16244
    b_y = 10150
    b_id = 42928
    b_xr = b_x-r_x
    b_yr = b_y-r_y
    rospy.init_node('laser_test', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback_lidar)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        #print("hey")
        w2.potential_occlusion_check(lidar_array,b_id,[r_x,r_y],[math.cos(theta_r),math.sin(theta_r)])



if __name__ == '__main__':
    main()
