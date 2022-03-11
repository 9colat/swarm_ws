#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from datetime import datetime
import numpy as np
import math
from filterpy.kalman import ExtendedKalmanFilter

wheel_speed = [0] * 4
IMU_data_acc = [0] * 3
IMU_data_gyro = [0] * 3
IMU_data_mag = [0] * 3


#collecting data neccessary for the kalman filter
def callback_IMU_acc(data): #from the accelometer
    global IMU_data_acc
    IMU_data_acc[0] = data.x
    IMU_data_acc[1] = data.y
    IMU_data_acc[2] = data.z

def callback_IMU_gyro(data): #from the gyroscope
    global IMU_data_acc
    IMU_data_gyro[0] = data.x
    IMU_data_gyro[1] = data.y
    IMU_data_gyro[2] = data.z

def callback_IMU_mag(data): #from the magnetometer
    global IMU_data_acc
    IMU_data_mag[0] = data.x
    IMU_data_mag[1] = data.y
    IMU_data_mag[2] = data.z

def callback_distance(data): #from the beacon
    global w1
    w1.updating_distance(data.ID, data.RSSI, data.distance)


#state propagation model
#x_k = f(x_k_previous,input) + process_noise

#output (measurement) model
#z_k = h(x_k) + measurement_noise

ekf_filter = ExtendedKalmanFilter(5, 3) # number of state vectors - position (x,y), velocity(x), heading(x,y'); measurement variables - mag_x, mag_y, beacon distance



def main():

    while not rospy.is_shutdown():
        rospy.Subscriber("imu_acc", Vector3, callback_IMU_acc)
        rospy.Subscriber("imu_gyro", Vector3, callback_IMU_gyro)
        rospy.Subscriber("imu_mag", Vector3, callback_IMU_mag)
        rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
        rate.sleep()


if __name__ == '__main__':
    main()
