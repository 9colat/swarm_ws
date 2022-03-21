#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
import time
from datetime import datetime
import numpy as np
import math
from autograd import jacobian

from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_discrete_white_noise


# constants
rotated_matrix = np.array([[0.0, -1.0], [1.0, 0.0]])
mag_x_calibrated = 0.0
mag_y_calibrated = 0.0
pi = 3.141593


class USPS_data:
    def __init__(self):

        #self stuff for beacons
        self.id = np.array([42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540])
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
        self.imu_acc = np.array([0.0]*3).reshape(-1,1)
        self.imu_gyro = np.array([0.0]*3).reshape(-1,1)
        self.imu_mag = np.array([0.0]*3).reshape(-1,1)
        self.old_time = 0.0
        self.heading = np.array([1.0]*2).reshape(-1,1)
        self.position = np.array([0.0]*3).reshape(-1,1)
        self.velocity = 0.0
        self.predicted_heading = np.array([1.0]*2).reshape(-1,1)
        self.predicted_position = np.array([0.0]*3).reshape(-1,1)
        self.predicted_velocity = 0.0
        self.input = np.array([0.0]*2)
        self.x_state = np.array([0.0]*3)
        self.measurement = 0.0
        self.measurement_estimated = 0.0



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
        delta_time = time.time() - self.old_time
        self.old_time = time.time()

        self.position = self.position + np.dot(self.velocity,self.heading) * delta_time
        self.velocity = self.velocity + self.imu_acc[0] * delta_time
        self.heading = self.heading + np.dot(rotated_matrix,self.heading) * self.imu_gyro[2] * delta_time
        self.state_model()

    def prediction_model(self):
        global local_position, local_velocity, local_heading
        delta_time = time.time() - self.old_time
        self.old_time = time.time()

        self.predicted_position = self.predicted_position + np.dot(self.predicted_velocity,self.predicted_heading) * delta_time
        self.predicted_velocity = self.predicted_velocity + self.imu_acc[0] * delta_time
        self.predicted_heading = self.predicted_heading + np.dot(rotated_matrix,self.heading) * self.imu_gyro[2] * delta_time
        self.predicted_state_model()



    def input_model(self):
        self.input = np.transpose([self.imu_acc[0], self.imu_gyro[2]])

    def state_model(self):
        self.x_state = np.transpose([np.transpose(self.position), self.velocity, np.transpose(self.heading)])

    def predicted_state_model(self):
        self.x_state_predicted = np.transpose([np.transpose(self.predicted_position), self.predicted_velocity, np.transpose(self.predicted_heading)])


    def extended_kalman_filter(self):

    self.measurement = pow(np.linalg.norm(self.position - BEACON), 2)
    self.measurement_estimated = pow((np.linalg.norm(self.predicted_position - BEACON), 2)
    measurement_difference = self.measurement - self.measurement_estimated

    B_index=rem(i,3)+1;
    Beacon=Beacons(:,B_index);
    #y=norm(states(1:2,i)-Beacon)^2+randn(1)*0.1;
    #ye=norm(e_states(1:2,i+1)-Beacon)^2;
    #yd=y-ye;
    H=HdNum(Beacon(1),Beacon(2),e_states(1,i),e_states(2,i));
    

    S=H*P*H'+R;
    K=P*H'*inv(S);
    e_states(:,i+1)=e_states(:,i+1)+K*yd;
    P=(eye(5)-K*H)*P;





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




#velocity = imu_acc[0]


def main():


    ekf_filter = ExtendedKalmanFilter(5, 3) # number of state vectors - position (x,y), velocity(x), heading(x,y'); measurement variables - mag_x, mag_y, beacon distance
    imu.updating_imu([1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,3.0,3.0])
    #print(local_position)
    #print(local_velocity)
    #print(local_heading)

    #print(rotated_matrix[1])
    #x=1
    #while True:

#        x+1

    #while not rospy.is_shutdown():
    #    rospy.Subscriber("imu_data", odom_and_imu, callback_imu)
    #    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    #    rate.sleep()


    #ekf_filter.x = imu.x_state     # initial state (location and velocity)
    #ekf_filter.F = np.array([[1.,1.], [0.,1.]])    # state transition matrix

    #ekf_filter.H = np.array([[1.,0.]])    # Measurement function - this is where we should pass beacons and IMU data

    #ekf_filter.P *= 1000.                 # covariance matrix
    #ekf_filter.R = 5                      # state uncertainty
    #ekf_filter.Q = Q_discrete_white_noise(2, dt, .1) # process uncertainty



    #while True:
    #ekf_filter.predict()
    #ekf_filter.update(get_some_measurement()) #input function - using_imu_data()?

    # do something with the output
    #x = ekf_filter.x
    #do_something_amazing(x) #output function # ?
    #print(rotated_matrix)
if __name__ == '__main__':
    main()
