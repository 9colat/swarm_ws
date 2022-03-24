#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
import time
from datetime import datetime
import numpy as np
import math
import random
#from autograd import jacobian
import matplotlib.pyplot as plt

#from filterpy.kalman import ExtendedKalmanFilter
#from filterpy.common import Q_discrete_white_noise


# constants
rotated_matrix = np.array([[0.0, -1.0], [1.0, 0.0]])
mag_x_calibrated = 0.0
mag_y_calibrated = 0.0
pi = 3.141593


class EKF:
    def __init__(self):

        #self stuff for beacons
        self.id = [42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.x = np.array([11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560])    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]
        self.y = np.array([5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549])    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
        self.z = np.array([5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767])    #[5577   , 5577  , 4286  , 3530  , 5578  , 5577  , 5577  , 5578  , 5578  , 5578  , 3767  , 3767  , 5577  ]
        self.count = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.distance = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.RSSI = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
        self.pose_est = np.array([16000.0, 6000.0, 300.0])
        self.pose_est_stored = np.array([[16000.0, 6000.0, 300.0],[16000.0, 6000.0, 300.0],[16000.0, 6000.0, 300.0]])
        self.pose_predict_from_pose = np.array([0.0, 0.0, 0.0])
        self.pose_meas_beacon = np.array([0.0, 0.0, 0.0])
        self.time_i = np.array([3.0, 2.0, 1.0])
        self.acc_meas = np.array([0.0, 0.0, 0.0])
        self.omega = np.array([0.0, 0.0])
        self.r = 0.04
        self.l = 0.229
        self.callibration_factor_acc = 1.0
        self.floor_corection_array = np.array([[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0]])
        self.floor_corection = np.array([0.0,0.0,0.0])

        # self stuff for IMU
        self.imu_acc = np.array([[0.0, 0.0, 0.0]]).T
        self.imu_gyro = np.array([[0.0, 0.0, 0.2]]).T
        self.imu_mag = np.array([[0.0, 0.0, 0.0]]).T
        self.old_time = time.time() / 1000
        self.predicted_old_time = time.time() / 1000
        self.heading = np.array([[1.0, 0.0]]).T
        self.position = np.array([[0.0, 0.0]]).T
        self.velocity = 1.0
        self.predicted_heading = np.array([[0.0, 0.0]]).T
        self.predicted_position = np.array([[0.0, 0.0]]).T
        self.predicted_velocity = 0.0
        self.input = np.array([[0.0, 0.0]])
        self.x_state = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        self.state_predicted = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        self.measurement = 0.0
        self.measurement_estimated = 0.0
        self.H = np.array([0.0]*5)
        self.F = np.array([0.0]*5)



    def updating_distance(self, id, rssi, distance):
        index_of_data = self.id.index(id)
        self.distance[index_of_data] = distance
        self.RSSI[index_of_data] = rssi
        self.count[index_of_data] = 3

    def updating_imu(self, imu_acc, imu_gyro, imu_mag):
        self.imu_acc = imu_acc
        self.imu_gyro = imu_gyro
        self.imu_mag = imu_mag


    #def input_model(self):
    #    self.input = np.transpose([self.imu_acc[0], self.imu_gyro[2]])


    def extended_kalman_filter(self, id, delta_time):

        #beacons
        current_beacon = self.id.index(id)
        BEACON = np.array([[self.x[current_beacon]], [self.y[current_beacon]]])

        #time
        #delta_time = time.time()/1000 - self.old_time
        #self.old_time = time.time()/1000

        #noise
        R = 0.01
        P = np.identity(5)
        Q = 100 * np.identity(5)


        #setup

        # state
        self.position = self.position + np.dot(self.velocity, self.heading) * delta_time
        self.velocity = self.velocity + self.imu_acc[0] * delta_time
        self.heading = self.heading + np.dot(rotated_matrix, self.heading) * self.imu_gyro[2] * delta_time


        # predicted state
        self.predicted_position = self.predicted_position + np.dot(self.predicted_velocity, self.predicted_heading) * delta_time
        self.predicted_velocity = self.predicted_velocity + self.imu_acc[0] * delta_time
        self.predicted_heading = self.predicted_heading + np.dot(rotated_matrix, self.predicted_heading) * self.imu_gyro[2] * delta_time

        self.state_predicted = np.transpose([np.array([self.predicted_position[0], self.predicted_position[1], self.predicted_velocity, self.predicted_heading[0], self.predicted_heading[1]])])

        # jacobians
        self.F = np.array([[1,0,delta_time*self.predicted_heading[0],delta_time*self.predicted_velocity,0],
             [0,1,delta_time*self.predicted_heading[1],0,delta_time*self.predicted_velocity],
             [0,0,1,0,0],
             [0,0,0,1,-delta_time*self.imu_gyro[2]],
             [0,0,0,delta_time*self.imu_gyro[2],1]])


        H_magnetometer = np.array([[0,0,0,self.imu_mag[0],self.imu_mag[1]],
                          [0,0,0,self.imu_mag[1],-self.imu_mag[0]]])


        self.H = np.array([-2 * np.absolute(self.x[current_beacon] - self.predicted_position[0]) * np.sign(self.x[current_beacon] - self.predicted_position[0]), -2 * np.absolute(self.y[current_beacon] - self.predicted_position[1]) * np.sign(self.y[current_beacon] - self.predicted_position[1]),0,0,0])



        #remember to remove noise after you are finished with debugging
        # measurements
        self.measurement = pow(np.linalg.norm(self.position - BEACON), 2) + random.random()
        self.measurement_estimated = pow(np.linalg.norm(self.predicted_position - BEACON), 2)
        estimation_difference = self.measurement - self.measurement_estimated

        # kalman magic
        P = np.dot(self.F, np.dot(P, np.transpose([self.F]))) + Q
        S = np.dot(np.dot(self.H, P), np.transpose([self.H])) + R
        K = P * np.transpose([self.H]) * pow(S, -1)
        print("here", K)
        self.state_predicted = self.state_predicted + np.dot(K, estimation_difference)
        print(self.state_predicted)
        # output update
        #self.position[0] = self.state_predicted[0]
        #self.position[1] = self.state_predicted[1]
        #self.velocity = self.state_predicted[2]
        #self.heading[0] = self.state_predicted[3]
        #self.heading[1] = self.state_predicted[4]

        #covariance update
        P = np.identity(5) - np.dot(np.dot(K, self.H), P)


        return self.state_predicted

# ROS collection
def callback_imu(data): #from the beacon
    global w1
    local_imu_acc = [data.imu_acc.x, data.imu_acc.y, data.imu_acc.z]
    local_imu_gyro = [data.imu_gyro.x, data.imu_gyro.y, data.imu_gyro.z]
    local_imu_mag = [data.imu_mag.x, data.imu_mag.y, data.imu_mag.z]
    w1.updating_imu(data.imu_acc, data.imu_gyro, data.imu_mag)



def callback_distance(data): #from the beacon
    global w1
    w1.updating_distance(data.ID, data.RSSI, data.distance)






w1 = EKF()



#velocity = imu_acc[0]


def main():
    n = 100
    test_array = [0.0]*n
    dT = 0.1
    arr = [42867, 42928,  42929]

    for x in range(n):
        id = w1.id.index(arr[x%3])
        new_position_x = w1.position[0] + w1.heading[0] * w1.velocity * dT
        new_position_y = w1.position[1] + w1.heading[1] * w1.velocity * dT
        dist = math.sqrt(pow(w1.x[id] - w1.position[0], 2) + pow(w1.y[id] - w1.position[1], 2))
        w1.updating_distance(arr[x%3], 1, dist)
        test_array[x] = w1.extended_kalman_filter(arr[x%3], dT)
        plt.scatter(test_array[x],x)
        #time.sleep(1)
    #t = np.arange(0., 5., 0.2)
    # red dashes, blue squares and green triangles

    #plt.xlim([499327316-100, 499327316+100])
    #print(test_array[n-1])
    plt.show()



    #while not rospy.is_shutdown():
    #    rospy.Subscriber("imu_data", odom_and_imu, callback_imu)
    #    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    #    rate.sleep()
    # evenly sampled time at 200ms intervals








if __name__ == '__main__':
    main()
