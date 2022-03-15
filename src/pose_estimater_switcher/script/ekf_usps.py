#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
import time
from datetime import datetime
import numpy as np
import math
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise



# constants
rotated_matrix_1straw = [0.0, -1.0]
rotated_matrix_2ndraw = [1.0, 0.0]
mag_x_calibrated = 0.0
mag_y_calibrated = 0.0
pi = 3.141593


class USPS_data:
    def __init__(self):
        self.id =   [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.x =    [11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]
        self.y =    [5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
        self.z =    [5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]    #[5577   , 5577  , 4286  , 3530  , 5578  , 5577  , 5577  , 5578  , 5578  , 5578  , 3767  , 3767  , 5577  ]
        self.count =     [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.distance = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.RSSI =     [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.pose_est = [16000.0, 6000.0, 300.0]
        self.pose_est_stored = [[16000.0, 6000.0, 300.0],[16000.0, 6000.0, 300.0],[16000.0, 6000.0, 300.0]]
        self.pose_predict_from_pose = [0.0, 0.0, 0.0]
        self.pose_meas_beacon = [0.0, 0.0, 0.0]
        self.time_i = [3.0, 2.0, 1.0]
        self.acc_meas = [0.0, 0.0, 0.0]
        self.omega = [0.0, 0.0]
        self.r = 0.04
        self.l = 0.229
        self.callibration_factor_acc = 1.0
        self.floor_corection_array = [[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]]
        self.floor_corection = [0.0,0.0,0.0]
        #self.time_unit_convertion_factor = 1

    def updating_distance(self, id, rssi, distance):
        index_of_data = self.id.index(id)
        #print(index_of_data)
        self.distance[index_of_data] = distance
        self.RSSI[index_of_data] = rssi
        self.count[index_of_data] = 3
        #print(self.distance[index_of_data])
        #print("im working")



class IMU_data:
    def __init__(self):
        self.imu_acc = [0.0]*3
        self.imu_gyro = [0.0]*3
        self.imu_mag = [0.0]*3
        self.old_time = 0
        self.old_heading = [0.0]*2
        self.position = [0.0]*3
        self.velocity = 0.0
        self.heading = [0.0]*2


    def updating_imu(self, imu_acc, imu_gyro, imu_mag):
        global imu_collected
        self.imu_acc = imu_acc
        self.imu_gyro = imu_gyro
        self.imu_mag = imu_mag
        imu_collected.using_imu_data(imu_acc, imu_gyro, imu_mag)


    def using_imu_data(self):
        delta_time = time.time() - self.old_time
        self.old_time = time.time()

        self.velocity = self.imu_acc[0] * delta_time
        self.old_heading = (math.atan2(self.imu_mag[0] - mag_y_calibrated, self.imu_mag[1] - mag_x_calibrated) * 180 / pi) * delta_time

        for x in range(len(self.old_heading)):
            self.heading[x] = rotated_matrix[x] * self.old_heading[x] * self.imu_gyro[2] * delta_time
            self.position[x] = self.velocity * self.heading[x] * delta_time



        return position, velocity, heading




# ROS collection
def callback_imu(data): #from the beacon
    global imu
    local_imu_acc = [data.imu_acc.x, data.imu_acc.y, data.imu_acc.z]
    local_imu_gyro = [data.imu_gyro.x, data.imu_gyro.y, data.imu_gyro.z]
    local_imu_mag = [data.imu_mag.x, data.imu_mag.y, data.imu_mag.z]
    imu.updating_imu(data.imu_acc, data.imu_gyro, data.imu_mag)



def callback_distance(data): #from the beacon
    global w1
    w1.updating_distance(data.ID, data.RSSI, data.distance)








w1 = USPS_data()
imu = IMU_data()
ekf_filter = ExtendedKalmanFilter(5, 3) # number of state vectors - position (x,y), velocity(x), heading(x,y'); measurement variables - mag_x, mag_y, beacon distance



#velocity = imu_acc[0]


def main():

    global ekf_filter, imu, w1

    imu.updating_imu([1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,3.0,3.0])
    imu.using_imu_data([1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,3.0,3.0])

    #x=1
    #while True:

#        x+1


    ekf_filter.x = np.array([[2.], [0.]])       # initial state (location and velocity)
    #ekf_filter.F = np.array([[1.,1.], [0.,1.]])    # state transition matrix
    #ekf_filter.H = np.array([[1.,0.]])    # Measurement function
    #ekf_filter.P *= 1000.                 # covariance matrix
    #ekf_filter.R = 5                      # state uncertainty
    #ekf_filter.Q = Q_discrete_white_noise(2, dt, .1) # process uncertainty

    #while not rospy.is_shutdown():
    #    rospy.Subscriber("imu_data", odom_and_imu, callback_imu)
    #    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    #    rate.sleep()

    #while True:
    #ekf_filter.predict()
    #ekf_filter.update(get_some_measurement()) #input function

    # do something with the output
    #x = ekf_filter.x
    #do_something_amazing(x) #output function

if __name__ == '__main__':
    main()
