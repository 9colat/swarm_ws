#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from custom_msgs.msg import odom_and_imu
from custom_msgs.msg import USPS_msgs
from datetime import datetime
from estimator_function import Pose_Calculator
import numpy as np
import math
import time

PC = Pose_Calculator()
old_time = datetime.now()
old_time_timestamp = datetime.timestamp(old_time)
time_measured = 0
iteriator = 0

USPS_pub = rospy.Publisher('USPS_pose_est', Pose, queue_size=10)



# missing beacons
# id, x, y, z
# 44050, 7825, 9999, 4286
# 44539, 1999, 10677, 3531


class USPS_data:
    def __init__(self):
        self.id =   [44539, 44050, 42867, 42928,  42929,  44530,  44531,  44532,  44533,  44534,  44535,  44536,  44537,  44538,  44540]
        self.x =    [1999, 7825, 11700, 16244,  7824,   2000,   21369,  26163,  26163,  31000,  35766,  35766,  40205,  40204,  16560]    #[11700  , 16244 , 7824  , 2000  , 21369 , 26163 , 26163 , 31000 , 35766 , 35766 , 40205 , 40204 , 16560 ]
        self.y =    [10677, 9999, 5999,  10150,  5726,   4499,   6534,   9939,   3699,   6519,   10012,  3522,   11684,  4363,   3549]    #[5999   , 10150 , 5726  , 4499  , 6534  , 9939  , 3699  , 6519  , 10012  , 10012 , 11684 , 4363  , 3549 ]
        self.z =    [3531, 4286, 5577,  5577,   4286,   3530,   5578,   5577,   5577,   5578,   5578,   5578,   3767,   3767,   3767]    #[5577   , 5577  , 4286  , 3530  , 5578  , 5577  , 5577  , 5578  , 5578  , 5578  , 3767  , 3767  , 5577  ]
        self.count =     [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.distance = [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.RSSI =     [0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.pose_est = [16000.0, 6000.0, 300.0]
        self.pose_est_stored = [[16000.0, 6000.0, 380.0],[16000.0, 6000.0, 310.0],[16000.0, 6000.0, 360.0]]
        self.pose_predict_from_pose = [0.0, 0.0, 0.0]
        self.time_i = [3.0, 2.0, 1.0]
        self.acc_meas = [0.0, 0.0, 0.0]
        self.omega = [0.0, 0.0]
        self.r = 0.04 #radius
        self.l = 0.229 #length
        self.callibration_factor_acc = [1, 1, 1]
        self.floor_corection_array = [[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0]]
        self.floor_corection = [0.0,0.0,0.0]
        #self.time_unit_convertion_factor = 1

    # kalman filter part
    def pose_predict(self, time):
        for j in range(len(self.pose_est_stored[1])):
            #below - what is our predicted pose based on the prior pose
            self.pose_predict_from_pose[j] =  self.pose_est[j] + ((self.pose_est_stored[0][j]-self.pose_est_stored[1][j])/(self.time_i[0]-self.time_i[1])) * time + 1/2*((((self.pose_est_stored[0][j]-self.pose_est_stored[1][j])/(self.time_i[0]-self.time_i[1]))-((self.pose_est_stored[1][j]-self.pose_est_stored[2][j])/(self.time_i[1]-self.time_i[2])))/(self.time_i[0]-self.time_i[2]))*pow(time,2)
        for i in range(len(self.time_i)-1,0,-1):
            #time update
            self.time_i[i] = self.time_i[i-1]
        self.time_i[0] = time
        return self.pose_predict_from_pose


    # below we just update the readings that are measured and then sent through the teensy to the RP


    # the method (function) below is the estimated pose pose_est output (pose_est = A*pose_predict + B*pose_measured + C*noise)
    # kalman filter part
    def pose_updater(self):
        for i in range(len(self.pose_est_stored)-1,0,-1):
            for j in range(len(self.pose_est_stored[i])):
                self.pose_est_stored[i][j] = self.pose_est_stored[i-1][j]
        for k in range(len(self.pose_est_stored[0])):
            self.pose_est_stored[0][k] = self.pose_est[k]


    def updating_distance(self, id, rssi, distance):
        index_of_data = self.id.index(id)
        self.distance[index_of_data] = distance
        self.RSSI[index_of_data] = rssi
        self.count[index_of_data] = 3
        #print(self.distance[index_of_data])
        #print("im working")


    def omega_update(self, omega_right, omega_left):
        self.omega[0] = omega_right
        self.omega[1] = omega_left

    def updating_acc(self, acc_x, acc_y, acc_z):
        self.acc_meas[0] = acc_x
        self.acc_meas[1] = acc_y
        self.acc_meas[2] = acc_z


    def velocity_cal(self):
        v = self.omega[0] * self.r + self.omega[1] * self.r
        return v


    # pose_measured model - kalman filter part
    def pose_estimator_henrik_method(self):

        id_array = [0] * 11
        x_array = [0] * 11
        y_array = [0] * 11

        print("hello there")
        id_array = [0] * len(self.id)
        beacon_coor = [[0]*3]*len(self.id)

        pose_esti = [0,0,0]
        dist_array = [0] * 11
        j = 0

        period = 1000
        dt = datetime.now()
        ts = datetime.timestamp(dt)
        for i in range(len(self.distance)):

            if self.count[i] > 0:
                id_array[j] = self.id[i]
                beacon_coor[j][0] = self.x[i]
                beacon_coor[j][1] = self.y[i]
                beacon_coor[j][2] = self.z[i]
                dist_array[j] = self.distance[i]
                j = j + 1

        for k in range(j):
            for q in range(3):
                m = dist_array[k] * (self.pose_est[q]-beacon_coor[k][q])/abs(math.sqrt(pow(self.pose_est[0]-beacon_coor[k][0],2)+pow(self.pose_est[1]-beacon_coor[k][1],2)+pow(self.pose_est[2]-beacon_coor[k][2],2)))#-math.sqrt(pow(beacon_coor[k][0],2)+pow(beacon_coor[k][1],2)+pow(beacon_coor[k][2],2)))
                print((self.pose_est[q]-beacon_coor[k][q]))
                print("self.pose_est[q]: ",self.pose_est[q])
                print("beacon_coor[k][q]: ",beacon_coor[k][q])
                #print("m: ",m)
                #print("|o-l|: ", abs(math.sqrt(pow(self.pose_est[0],2)+pow(self.pose_est[1],2)+pow(self.pose_est[2],2))-math.sqrt(pow(beacon_coor[k][0],2)+pow(beacon_coor[k][1],2)+pow(beacon_coor[k][2],2))))
                pose_esti[q] = beacon_coor[k][q] + m

            #dist_new = math.sqrt(pow(self.pose_est[0] - pose_esti[0],2)+pow(self.pose_est[1] - pose_esti[1],2)+pow(self.pose_est[2] - pose_esti[2],2))
            #dp = mathpose_esti[0] = .sqrt(pow(self.pose_est[0] - x_array[k], 2) + pow(self.pose_est[1] - y_array[k], 2) + pow(self.pose_est[2] - z_array[k], 2))
            #print("dp: ",dp)
            #print("calculated: ",])
            #alpha = (dp - dist_array[k])/(dp + 10)
            #print("Alpha: ", alpha)
            #pose_esti[0] = self.pose_est[0] + alpha * x_array[k] - self.pose_est[0]
            #pose_esti[1] = self.pose_est[1] + alpha * y_array[k] - self.pose_est[1]
            #pose_esti[2] = self.pose_est[2] + alpha * z_array[k] - self.pose_est[2]
            #print(pose_esti[0])

            #dist_new = math.sqrt(pow(self.pose_est[0] - pose_esti[0], 2) + pow(self.pose_est[1] - pose_esti[1], 2) + pow(self.pose_est[2] - pose_esti[2], 2))

            #if dist_new <= 1:
            #    for c in range(len(pose_esti)):
            #        pose_esti[c] = beacon_coor[k][c] + 0.2 * m

                #pose_esti[0] = self.pose_est[0] + 0.2 * (x_array[k] - pose_esti[0])/dp
                #pose_esti[1] = self.pose_est[1] + 0.2 * (y_array[k] - pose_esti[1])/dp


        #add sort array of the maybe sorted by the time elapsed since it was set
        self.pose_est[0] = pose_esti[0]
        self.pose_est[1] = pose_esti[1]
        self.pose_est[2] = pose_esti[2]

        USPS_pose = Pose()
        USPS_pose.position.x = pose_esti[0]
        USPS_pose.position.y = pose_esti[1]
        USPS_pose.position.z = pose_esti[2]
        USPS_pub.publish(USPS_pose)
        #print(pose_esti)
        return pose_est


    # no touch
    def pose_estimator_trilatertion(self):
        id_array = [0] * 11
        x_array = [0] * 11
        x_sort_array = [0] * 4
        y_array = [0] * 11
        y_sort_array = [0] * 4
        z_array = [0] * 11
        z_sort_array = [0] * 4
        pose_est = [0,0,0]
        dist_array = [0] * 11
        j = 0
        period = 1000
        dt = datetime.now()
        ts = datetime.timestamp(dt)
        for i in range(len(self.distance)):
            if self.count[i] > 0:
                if self.count[i] < ts+period:
                    id_array[j] = self.id[i]
                    x_array[j] = self.x[i]
                    y_array[j] = self.y[i]
                    z_array[j] = self.z[i]
                    dist_array[j] = self.distance[i]
                    j = j + 1

        dist_sort = sorted(dist_array, reverse=True)
        x_sort_array[0] = x_array[dist_array.index(dist_sort[0])]
        y_sort_array[0] = y_array[dist_array.index(dist_sort[0])]
        z_sort_array[0] = z_array[dist_array.index(dist_sort[0])]
        if dist_sort[0] == dist_sort[1]:
            x_sort_array[1] = x_array[dist_array.index(dist_sort[1],dist_array.index(dist_sort[0])+1, len(x_array))]
            y_sort_array[1] = y_array[dist_array.index(dist_sort[1],dist_array.index(dist_sort[0])+1, len(y_array))]
            z_sort_array[1] = z_array[dist_array.index(dist_sort[1],dist_array.index(dist_sort[0])+1, len(z_array))]
        if dist_sort[0] != dist_sort[1]:
            x_sort_array[1] = x_array[dist_array.index(dist_sort[1])]
            y_sort_array[1] = y_array[dist_array.index(dist_sort[1])]
            z_sort_array[1] = z_array[dist_array.index(dist_sort[1])]
        if dist_sort[0] == dist_sort[2]:
            x_sort_array[2] = x_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[0])+1, len(x_array))]
            y_sort_array[2] = y_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[0])+1, len(y_array))]
            z_sort_array[2] = z_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[0])+1, len(z_array))]
        if dist_sort[1] == dist_sort[2]:
            x_sort_array[2] = x_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[1])+1, len(x_array))]
            y_sort_array[2] = y_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[1])+1, len(y_array))]
            z_sort_array[2] = z_array[dist_array.index(dist_sort[2],dist_array.index(dist_sort[1])+1, len(z_array))]
        if dist_sort[0] != dist_sort[2] and dist_sort[1] != dist_sort[2]:
            x_sort_array[2] = x_array[dist_array.index(dist_sort[2])]
            y_sort_array[2] = y_array[dist_array.index(dist_sort[2])]
            z_sort_array[2] = z_array[dist_array.index(dist_sort[2])]
        if dist_sort[0] == dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[0])+1, len(x_array))]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[0])+1, len(y_array))]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[0])+1, len(z_array))]
        if dist_sort[1] == dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[1])+1, len(x_array))]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[1])+1, len(y_array))]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[1])+1, len(z_array))]
        if dist_sort[2] == dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[2])+1, len(x_array))]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[2])+1, len(y_array))]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[3],dist_array.index(dist_sort[2])+1, len(z_array))]
        if dist_sort[0] != dist_sort[3] and dist_sort[1] != dist_sort[3] and dist_sort[2] != dist_sort[3]:
            x_sort_array[3] = x_array[dist_array.index(dist_sort[2])]
            y_sort_array[3] = y_array[dist_array.index(dist_sort[2])]
            z_sort_array[3] = z_array[dist_array.index(dist_sort[2])]

        m_1 = np.array([[x_sort_array[1]-x_sort_array[0],y_sort_array[1]-y_sort_array[0],z_sort_array[1]-z_sort_array[0]],[x_sort_array[2]-x_sort_array[1],y_sort_array[2]-y_sort_array[1],z_sort_array[2]-z_sort_array[1]],[x_sort_array[3]-x_sort_array[0],y_sort_array[3]-y_sort_array[0],z_sort_array[3]-z_sort_array[0]]])
        #print(m_1)
        m_1t = m_1.transpose()
        m_2 = np.array([[pow(x_sort_array[1],2)-pow(x_sort_array[0],2)+pow(y_sort_array[1],2)-pow(y_sort_array[0],2)+pow(z_sort_array[1],2)-pow(z_sort_array[0],2)],[pow(x_sort_array[2],2)-pow(x_sort_array[1],2)+pow(y_sort_array[2],2)-pow(y_sort_array[1],2)+pow(z_sort_array[2],2)-pow(z_sort_array[1],2)],[pow(x_sort_array[3],2)-pow(x_sort_array[0],2)+pow(y_sort_array[3],2)-pow(y_sort_array[0],2)+pow(z_sort_array[3],2)-pow(z_sort_array[0],2)]])
        m_pose = (1/2) * m_1t.dot(m_2)
        #self.pose[0] = m_pose[0]
        #self.pose[1] = m_pose[1]
        #self.pose[2] = m_pose[2]
        #print(m_pose)
        return m_pose








    # sensor fusion

    def measured_model_imu_and_odometry(self):
        global time_measured
        dt = datetime.now()
        old_time_measured = time_measured
        time_measured = datetime.timestamp(dt)
        time = time_measured - old_time_measured
        pose_predict_meas = [0,0,0]
        pose_predict_meas[0] = self.pose_est[0] + self.velocity_cal() * math.cos(((self.l*(self.omega[0] - self.omega[1])* self.r)/2)*time)*time + 1/2 * self.acc_meas[0] * pow(time,2)
        pose_predict_meas[1] = self.pose_est[1] + self.velocity_cal() * math.sin(((self.l*(self.omega[0] - self.omega[1])* self.r)/2)*time)*time + 1/2 * self.acc_meas[1] * pow(time,2)
        pose_predict_meas[2] = self.pose_est[2] + self.velocity_cal() * math.sin(((self.l*(self.omega[0] - self.omega[1])* self.r)/2)*time)*time + 1/2 * self.acc_meas[1] * pow(time,2)
        return pose_predict_meas

w1 = USPS_data()


def callback_distance(data):
    global w1, PC
    pose_meas_beacon = PC.pose_estimator(data.ID, data.distance)
    #if data.ID in w1.id:
        #print(data.ID, ": ", data.distance)
        #w1.updating_distance(data.ID, data.RSSI, data.distance)
    print(pose_meas_beacon)

def callback_odom_and_imu(data):
    global w1, iteriator
    if iteriator < 10:
        w1.updating_acc(w1.callibration_factor_acc[0]*data.imu_acc.x,w1.callibration_factor_acc[1]*data.imu_acc.y,w1.callibration_factor_acc[2]*data.imu_acc.z)
        w1.omega_update(data.omega_right, data.omega_left)
        for j in range(len(w1.floor_corection)):
            w1.floor_corection_array[j][iteriator] = w1.acc_meas[j]
            w1.floor_corection[j] = w1.floor_corection_array[j][iteriator]/(iteriator+1)
        print(w1.floor_corection_array)
        iteriator += 1
    if iteriator >= 10:
        w1.updating_acc((w1.callibration_factor_acc[0]*data.imu_acc.x) - w1.floor_corection[0],(w1.callibration_factor_acc[1]*data.imu_acc.y) - w1.floor_corection[1],(w1.callibration_factor_acc[2]*data.imu_acc.z) - w1.floor_corection[2])
        w1.omega_update(data.omega_right, data.omega_left)
    #print("im in acc and odom the callback")

def pose_estimator():
    global w1
    #print(w1.pose_predict(10))
    #w1.pose_estimator_trilatertion()
    #dist_sort = sorted(w1.distance, reverse=True)
    #print(w1.distance[self.id.index(44532)])
    #print(w1.pose_estimator_henrik_method())
    #i = [1,2,3]
    #print(i[1])
    rospy.init_node('USPS_pose_estimator', anonymous=True)
    #rospy.Subscriber("odometry_and_IMU", odom_and_imu, callback_odom_and_imu)
    rospy.Subscriber("beacon_data", USPS_msgs, callback_distance)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():

        #print("ola ")
        w1.pose_estimator_henrik_method()

        #plotter(i,1,1)
        #print(i)


        #print("somewhat ")
        rate.sleep()

if __name__ == '__main__':
    pose_estimator()
