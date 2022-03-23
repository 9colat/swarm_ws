#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
import time
from datetime import datetime
import numpy as np
import math
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
        self.imu_acc = np.array([[0.0]*3])
        self.imu_gyro = np.array([[0.0]*3])
        self.imu_mag = np.array([[0.0]*3])
        self.old_time = time.time() / 1000
        self.predicted_old_time = time.time() / 1000
        self.heading = np.array([[0.0]*2])
        self.position = np.array([[0.0]*2])
        self.velocity = 0.0
        self.predicted_heading = np.array([[0.0]*2])
        self.predicted_position = np.array([[0.0]*2])
        self.predicted_velocity = 0.0
        self.input = np.array([[0.0]*2])
        self.x_state = np.array([[0.0]*5])
        self.x_state_predicted = np.array([[0.0]*5])
        self.measurement = 0.0
        self.measurement_estimated = 0.0
        self.H = np.array([0.0]*5)



    def updating_distance(self, id, rssi, distance):
        index_of_data = self.id.index(id)
        self.distance[index_of_data] = distance
        self.RSSI[index_of_data] = rssi
        self.count[index_of_data] = 3

    def updating_imu(self, imu_acc, imu_gyro, imu_mag):
        self.imu_acc = imu_acc
        self.imu_gyro = imu_gyro
        self.imu_mag = imu_mag
        self.using_imu_data()
        self.prediction_model()
        self.input_model()



    def using_imu_data(self):
        delta_time = time.time()/1000 - self.old_time
        self.old_time = time.time()/1000

        self.position = self.position + np.dot(self.velocity, self.heading) * delta_time
        self.velocity = self.velocity + self.imu_acc[0] * delta_time
        self.heading = self.heading + np.dot(rotated_matrix, np.transpose(self.heading)) * self.imu_gyro[2] * delta_time

        self.state_model()




    def prediction_model(self):
        delta_time = time.time()/1000 - self.predicted_old_time
        self.predicted_old_time = time.time()/1000

        self.predicted_position = self.predicted_position + np.dot(self.predicted_velocity,self.predicted_heading) * delta_time
        self.predicted_velocity = self.predicted_velocity + self.imu_acc[0] * delta_time
        self.predicted_heading = self.predicted_heading + np.dot(rotated_matrix, np.transpose(self.predicted_heading)) * self.imu_gyro[2] * delta_time
        self.predicted_state_model()



    def input_model(self):
        self.input = np.transpose([self.imu_acc[0], self.imu_gyro[2]])

    def state_model(self):
        self.x_state = np.transpose([np.transpose(self.position), self.velocity, np.transpose(self.heading)])


    def predicted_state_model(self):
        self.x_state_predicted = np.transpose([np.transpose(self.predicted_position), self.predicted_velocity, np.transpose(self.predicted_heading)])



    def extended_kalman_filter(self, id=44531):

        current_beacon = self.id.index(id)
        delta_time = time.time()/1000 - self.old_time
        self.old_time = time.time()/1000
        R = 0.1
        P = np.identity(5)
        # jacobians
        F = np.array([[1,0,delta_time*self.heading[0][0],delta_time*self.velocity,0],
             [0,1,delta_time*self.heading[0][1],0,delta_time*self.velocity],
             [0,0,1,0,0],
             [0,0,0,1,-delta_time*self.imu_gyro[2]],
             [0,0,0,delta_time*self.imu_gyro[2],1]])
        H_magnetometer = np.array([[0,0,0,self.imu_mag[0],self.imu_mag[1]],
                          [0,0,0,self.imu_mag[1],-self.imu_mag[0]]])
        self.H = np.array([-2 * np.absolute(self.x[current_beacon] - self.predicted_position[0][0]) * np.sign(self.x[current_beacon] - self.predicted_position[0][0]), -2 * np.absolute(self.y[current_beacon] - self.predicted_position[0][1]) * np.sign(self.y[current_beacon] - self.predicted_position[0][1]),0,0,0])
        BEACON = np.array([[self.x[current_beacon]], [self.y[current_beacon]]])

        self.measurement = pow(np.linalg.norm(self.position - BEACON), 2)
        self.measurement_estimated = pow(np.linalg.norm(self.predicted_position - BEACON), 2)
        estimation_difference = self.measurement - self.measurement_estimated

        S = np.dot(np.dot(self.H, P), np.transpose([self.H])) + R
        K = P * np.transpose([self.H]) * pow(S, -1)


        #print(len(np.array([[1.0]*2])) , len(np.array([[1.0]*2])[0]))
        #print("H: ", len(self.H))
        #print("P: ", P)
        #print("Transposed H: ", self.H.T)


        #S=H*P*H'+R;
        #K=P*H'*inv(S);
        #e_states(:,i+1)=e_states(:,i+1)+K*yd;
        #P=(eye(5)-K*H)*P;
        #self.predicted_position = self.predicted_position + np.dot(K, estimation_difference)
        #self.predicted_velocity = self.predicted_velocity + np.dot(K, estimation_difference)
        #self.predicted_heading = self.predicted_heading + np.dot(K, estimation_difference)
        #P = (np.identity(5) - np.dot(np.dot(K, self.H)), P)

        return self.measurement



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

    for x in range(n):
        w1.updating_imu([11.0,50.0,20.0],[10.0,50.0,2.0],[12.0,41.0,5.0])
        w1.updating_distance(44531, 1, 10000)

        #print(w1.measurement)
        #print(w1.measurement_estimated)
        test_array[x] = w1.extended_kalman_filter()
        plt.scatter(test_array[x],x)
        #print(w1.x_state_predicted)
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
