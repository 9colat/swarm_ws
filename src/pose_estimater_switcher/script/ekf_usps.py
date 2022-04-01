#!/usr/bin/env python3
import numpy as np
import math


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
        self.imu_gyro = np.array([[0.0, 0.0, 0.1]]).T
        self.imu_mag = np.array([[0.0, 0.0, 0.0]]).T
        self.predicted_heading = np.array([[1.0, 0.0]]).T
        self.predicted_position = np.array([[10.0, 15.0]]).T
        self.predicted_velocity = 1.0
        self.input = np.array([[0.0, 0.0]])
        self.state_predicted = np.array([[float(self.predicted_position[0]), float(self.predicted_position[1]), self.predicted_velocity, float(self.predicted_heading[0]), float(self.predicted_heading[1])]])
        self.measurement = 0.0
        self.measurement_estimated = 0.0
        self.H_beacon = np.array([0.0]*5)
        self.H_magnetometer = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]*2)
        self.F = np.array([0.0]*5)



    def updating_imu(self, imu_acc, imu_gyro):
        self.imu_acc = imu_acc
        self.imu_gyro = imu_gyro

    def angle_to_vector(self, theta):
        vector = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        return vector


    #def input_model(self):
    #    self.input = np.transpose([self.imu_acc[0], self.imu_gyro[2]])


    def state_prediction(self, delta_time):

        # predicted state
        self.predicted_position = self.predicted_position + np.dot(self.predicted_velocity, self.predicted_heading) * delta_time
        self.predicted_velocity = float(self.predicted_velocity + self.imu_acc[0] * delta_time)
        self.predicted_heading = self.predicted_heading + np.dot(rotated_matrix, self.predicted_heading) * self.imu_gyro[2] * delta_time

        # combined together so it can be multiplied later
        self.state_predicted = np.array([[float(self.predicted_position[0]), float(self.predicted_position[1]), self.predicted_velocity, float(self.predicted_heading[0]), float(self.predicted_heading[1])]]).T

        return self.state_predicted



    def magnetometer_measurement_updater_EKF(self, imu_mag, delta_time):

        self.state_prediction(delta_time)

        global_to_local_mag = np.dot(self.angle_to_vector(3*math.pi/180), np.array([[1, 0]]).T)

        # noise
        R = 0.01
        P = np.identity(5)
        Q = 100 * np.identity(5)


        # jacobians
        self.F = np.array([[1, 0, float(delta_time * self.predicted_heading[0]), float(delta_time * self.predicted_velocity), 0],
             [0, 1, float(delta_time * self.predicted_heading[1]), 0, float(delta_time * self.predicted_velocity)],
             [0, 0, 1, 0, 0],
             [0, 0, 0, 1, float(-delta_time * self.imu_gyro[2])],
             [0, 0, 0, float(delta_time*self.imu_gyro[2]), 1]])


        self.H_magnetometer = np.array([[0, 0, 0, float(imu_mag[0]), float(imu_mag[1])],
                          [0, 0 , 0, float(imu_mag[1]), float(-imu_mag[0])]])


        #remember to remove noise after you are finished with debugging
        # measurements based on the magnetometer data
        self.measurement = imu_mag
        self.measurement_estimated = np.array([[float(np.dot(global_to_local_mag.T, self.predicted_heading))], [float((np.dot(np.dot(global_to_local_mag.T, rotated_matrix), self.predicted_heading)))]])
        #print(math.degrees(math.atan2(self.measurement_estimated[1], self.measurement_estimated[0])))
        estimation_difference = self.measurement - self.measurement_estimated
        #print("R: ", rotated_matrix)
        #print("H: ", self.predicted_heading)
        #print("Dot product: ", np.dot(rotated_matrix, self.predicted_heading))
        #print("Angle: ", math.degrees(math.atan2(self.measurement[1], self.measurement[0])))

        # kalman magic
        P = np.dot(np.dot(self.F, P), np.transpose(self.F)) + Q
        S = np.dot(np.dot(self.H_magnetometer, P), np.transpose(self.H_magnetometer)) + R
        K = np.dot(np.dot(P, np.transpose(self.H_magnetometer)), np.linalg.inv(S))

        # output update
        self.state_predicted = self.state_predicted + np.dot(K, estimation_difference)
        self.predicted_position[0] = self.state_predicted[0]
        self.predicted_position[1] = self.state_predicted[1]
        self.predicted_velocity = float(self.state_predicted[2])
        self.predicted_heading[0] = np.linalg.norm(self.state_predicted[3])
        self.predicted_heading[1] = np.linalg.norm(self.state_predicted[4])

        #covariance update
        P = np.dot(np.identity(5) - (np.dot(K, self.H_magnetometer)), P)


        return self.state_predicted


    def beacon_measurement_updater_EKF(self, id, DISTANCE, delta_time):


        self.state_prediction(delta_time)

        #beacons
        current_beacon = self.id.index(id)
        BEACON = np.array([[self.x[current_beacon]], [self.y[current_beacon]]])


        # noise
        R = 0.01
        P = np.identity(5)
        Q = 100 * np.identity(5)


        # jacobians
        self.F = np.array([[1, 0, float(delta_time * self.predicted_heading[0]), float(delta_time * self.predicted_velocity), 0],
             [0, 1, float(delta_time * self.predicted_heading[1]), 0, float(delta_time * self.predicted_velocity)],
             [0, 0, 1, 0, 0],
             [0, 0, 0, 1, float(-delta_time * self.imu_gyro[2])],
             [0, 0, 0, float(delta_time*self.imu_gyro[2]), 1]])


        self.H_beacon = np.array([[float(-2 * np.absolute(self.x[current_beacon] - self.predicted_position[0]) * np.sign(self.x[current_beacon] - self.predicted_position[0])), float(-2 * np.absolute(self.y[current_beacon] - self.predicted_position[1]) * np.sign(self.y[current_beacon] - self.predicted_position[1])), 0, 0, 0]])


        #remember to remove noise after you are finished with debugging
        # measurements based on the beacon readings
        self.measurement = pow(DISTANCE, 2)
        self.measurement_estimated = pow(np.linalg.norm(self.predicted_position - BEACON), 2)
        estimation_difference = self.measurement - self.measurement_estimated

        # kalman magic
        P = np.dot(np.dot(self.F, P), np.transpose(self.F)) + Q
        S = np.dot(np.dot(self.H_beacon, P), np.transpose(self.H_beacon)) + R
        K = np.dot(np.dot(P, np.transpose(self.H_beacon)), np.linalg.inv(S))

        # output update
        self.state_predicted = self.state_predicted + np.dot(K, estimation_difference)
        self.predicted_position[0] = self.state_predicted[0]
        self.predicted_position[1] = self.state_predicted[1]
        self.predicted_velocity = float(self.state_predicted[2])
        self.predicted_heading[0] = self.state_predicted[3]
        self.predicted_heading[1] = self.state_predicted[4]


        #covariance update
        P = np.dot(np.identity(5) - (np.dot(K, self.H_beacon)), P)

        return self.state_predicted
